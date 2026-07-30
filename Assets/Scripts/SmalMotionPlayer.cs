using System;
using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// Drives the InterPet4D SMAL neutral rig (bones "SMAL_joint_00".."SMAL_joint_34")
    /// from a clip exported by <c>Python/export_smal_unity_clip.py</c>, at runtime and
    /// without any baked animation.
    ///
    /// The clip stores, per frame, each joint's world-space rotation delta from the
    /// neutral (rest) pose in SMAL/armature space, plus the global body transform
    /// (R_world / t_world / s_world). SMAL and Unity use different coordinate systems
    /// (SMAL is right-handed Z-up; Unity is left-handed Y-up) and the FBX import adds
    /// its own axis/scale conversion. Rather than hard-code that conversion, this
    /// component recovers it at startup by fitting the exported rest joint positions
    /// to the imported rig's actual bone positions (an affine similarity solve). Every
    /// pose delta is then conjugated through that recovered map, so the same clip works
    /// regardless of how the FBX was imported.
    ///
    /// Attach this to the root of the imported dog (the object that contains the
    /// SMAL_joint bones and the SkinnedMeshRenderer) and assign the exported JSON.
    /// </summary>
    public class SmalMotionPlayer : MonoBehaviour
    {
        [Header("Clip")]
        [Tooltip("JSON produced by Python/export_smal_unity_clip.py.")]
        [SerializeField] TextAsset clipJson;

        [Header("Rig")]
        [Tooltip("Root that contains the SMAL_joint bones. Defaults to this transform.")]
        [SerializeField] Transform modelRoot;

        [Header("Playback")]
        [Tooltip("0 = use the clip's own fps.")]
        [SerializeField] float fpsOverride;
        [SerializeField] bool loop = true;
        [Tooltip("Apply the captured global body motion (turn / translate / scale). " +
                 "Disable to keep the dog planted and only play the articulation.")]
        [SerializeField] bool applyGlobalMotion = true;

        [Header("Debug")]
        [Tooltip("Log the startup calibration fit error (metres). A large value means " +
                 "the rig bones did not match the exported rest skeleton.")]
        [SerializeField] bool logCalibration = true;

        [Serializable]
        class Clip
        {
            public string name;
            public int fps;
            public int frameCount;
            public int jointCount;
            public string boneNamePrefix;
            public int[] parents;
            public float[] restJoints; // jointCount * 3
            public float[] deltas;     // frameCount * jointCount * 4  (x,y,z,w)
            public float[] rootRot;    // frameCount * 4               (x,y,z,w)
            public float[] rootPos;    // frameCount * 3
            public float[] rootScale;  // frameCount
        }

        Clip clip;
        Transform[] bones;
        Quaternion[] boneRestInRoot;   // bone rest orientation relative to modelRoot
        Quaternion[] boneRestLocal;    // bone rest local rotation (relative to its parent)

        // Recovered SMAL(arm) -> modelRoot-local map.  A = scale * R (linear), b = offset.
        Matrix4x4 armToRoot;           // 4x4: [A | b]
        Matrix4x4 armToRootInverse;
        Matrix4x4 rotArmToRoot;        // R_M  (linear only, translation 0)
        Matrix4x4 rotRootToArm;        // R_M^T

        // modelRoot's scene placement captured at bind (global motion composes onto it).
        Matrix4x4 bindLocal;

        float time;

        void Awake()
        {
            if (modelRoot == null)
                modelRoot = transform;

            if (clipJson == null)
                throw new MissingReferenceException("SmalMotionPlayer: no clip JSON assigned");

            clip = JsonUtility.FromJson<Clip>(clipJson.text);
            if (clip == null || clip.frameCount <= 0 || clip.jointCount <= 0)
                throw new InvalidOperationException("SmalMotionPlayer: clip JSON failed to parse");

            ResolveBones();
            CaptureBind();
            Calibrate();

            // Never let the mesh get culled while we push bones far from the origin.
            foreach (var smr in modelRoot.GetComponentsInChildren<SkinnedMeshRenderer>())
                smr.updateWhenOffscreen = true;
            // Disable any imported Animator so it doesn't fight the manual posing.
            foreach (var animator in modelRoot.GetComponentsInChildren<Animator>())
                animator.enabled = false;

            Pose(0);
        }

        void ResolveBones()
        {
            bones = new Transform[clip.jointCount];
            var all = modelRoot.GetComponentsInChildren<Transform>(true);
            for (int j = 0; j < clip.jointCount; j++)
            {
                string wanted = $"{clip.boneNamePrefix}{j:00}";
                foreach (var t in all)
                {
                    if (t.name == wanted) { bones[j] = t; break; }
                }
                if (bones[j] == null)
                    throw new MissingReferenceException(
                        $"SmalMotionPlayer: bone '{wanted}' not found under {modelRoot.name}");
            }
        }

        void CaptureBind()
        {
            bindLocal = Matrix4x4.TRS(
                modelRoot.localPosition, modelRoot.localRotation, modelRoot.localScale);

            boneRestInRoot = new Quaternion[clip.jointCount];
            boneRestLocal = new Quaternion[clip.jointCount];
            Quaternion invRoot = Quaternion.Inverse(modelRoot.rotation);
            for (int j = 0; j < clip.jointCount; j++)
            {
                boneRestInRoot[j] = invRoot * bones[j].rotation;
                // Captured before Pose(0): the joint's neutral (rest) local rotation.
                boneRestLocal[j] = bones[j].localRotation;
            }
        }

        /// <summary>The neutral (rest) local rotation of SMAL joint
        /// <paramref name="joint"/> — the pose the clip's per-frame deltas are measured
        /// against — captured before any frame is applied. Retargeters that copy a
        /// joint's articulation use this as the reference so the copy is relative to the
        /// true neutral, not a particular animation frame. Valid after Awake.</summary>
        public bool TryGetRestLocalRotation(int joint, out Quaternion rest)
        {
            if (boneRestLocal != null && joint >= 0 && joint < boneRestLocal.Length)
            {
                rest = boneRestLocal[joint];
                return true;
            }
            rest = Quaternion.identity;
            return false;
        }

        /// <summary>
        /// Fit the affine map (A, b) with bonePos_in_root ≈ A * restJoint + b from the
        /// exported rest skeleton to the imported bone positions, then split A into a
        /// uniform scale and an orthonormal rotation used to convert pose deltas.
        /// </summary>
        void Calibrate()
        {
            int n = clip.jointCount;
            var arm = new Vector3[n];
            var root = new Vector3[n];
            for (int j = 0; j < n; j++)
            {
                arm[j] = new Vector3(
                    clip.restJoints[j * 3 + 0],
                    clip.restJoints[j * 3 + 1],
                    clip.restJoints[j * 3 + 2]);
                root[j] = modelRoot.InverseTransformPoint(bones[j].position);
            }

            // Normal equations for each output axis: minimise ||root_k - (A_k·arm + b_k)||.
            // Unknowns per axis: [ax, ay, az, b].  N (4x4) is shared across axes.
            double[,] N = new double[4, 4];
            double[,] rhs = new double[4, 3];
            for (int j = 0; j < n; j++)
            {
                double[] p = { arm[j].x, arm[j].y, arm[j].z, 1.0 };
                for (int r = 0; r < 4; r++)
                {
                    for (int c = 0; c < 4; c++) N[r, c] += p[r] * p[c];
                    rhs[r, 0] += p[r] * root[j].x;
                    rhs[r, 1] += p[r] * root[j].y;
                    rhs[r, 2] += p[r] * root[j].z;
                }
            }
            double[,] sol = SolveLinear(N, rhs); // 4x3 : rows = [ax,ay,az,b], cols = axis

            armToRoot = Matrix4x4.identity;
            for (int axis = 0; axis < 3; axis++)
            {
                armToRoot[axis, 0] = (float)sol[0, axis];
                armToRoot[axis, 1] = (float)sol[1, axis];
                armToRoot[axis, 2] = (float)sol[2, axis];
                armToRoot[axis, 3] = (float)sol[3, axis];
            }
            armToRootInverse = armToRoot.inverse;

            // A = s * R.  Recover s from the mean column length, R = A / s.
            float sx = new Vector3(armToRoot[0, 0], armToRoot[1, 0], armToRoot[2, 0]).magnitude;
            float sy = new Vector3(armToRoot[0, 1], armToRoot[1, 1], armToRoot[2, 1]).magnitude;
            float sz = new Vector3(armToRoot[0, 2], armToRoot[1, 2], armToRoot[2, 2]).magnitude;
            float scale = (sx + sy + sz) / 3f;

            rotArmToRoot = Matrix4x4.identity;
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    rotArmToRoot[r, c] = armToRoot[r, c] / scale;
            rotRootToArm = rotArmToRoot.transpose;

            if (logCalibration)
            {
                float maxErr = 0f;
                for (int j = 0; j < n; j++)
                    maxErr = Mathf.Max(maxErr, (armToRoot.MultiplyPoint3x4(arm[j]) - root[j]).magnitude);
                Debug.Log($"SmalMotionPlayer: calibrated {clip.name} " +
                          $"(scale={scale:F4}, max rest fit error={maxErr:F5} m)");
            }
        }

        void Update()
        {
            float fps = fpsOverride > 0f ? fpsOverride : clip.fps;
            time += Time.deltaTime;
            float framePos = time * fps;
            int f = Mathf.FloorToInt(framePos);
            float frac = framePos - f;
            int next;
            if (loop)
            {
                f = ((f % clip.frameCount) + clip.frameCount) % clip.frameCount;
                next = (f + 1) % clip.frameCount;
            }
            else
            {
                f = Mathf.Min(f, clip.frameCount - 1);
                next = Mathf.Min(f + 1, clip.frameCount - 1);
                if (f == next)
                    frac = 0f;
            }
            Pose(f, next, frac);
        }

        void Pose(int f) => Pose(f, f, 0f);

        void Pose(int f, int next, float frac)
        {
            ApplyGlobal(f, next, frac);

            Quaternion rootRot = modelRoot.rotation;
            int baseF = f * clip.jointCount * 4;
            int baseN = next * clip.jointCount * 4;
            for (int j = 0; j < clip.jointCount; j++) // parents precede children in SMAL order
            {
                int i = baseF + j * 4;
                var deltaArm = new Quaternion(
                    clip.deltas[i + 0], clip.deltas[i + 1], clip.deltas[i + 2], clip.deltas[i + 3]);
                // Slerp the per-joint world delta between whole frames so the pose is
                // not stepped at low clip fps.
                if (frac > 0f)
                {
                    int n = baseN + j * 4;
                    var deltaNext = new Quaternion(
                        clip.deltas[n + 0], clip.deltas[n + 1], clip.deltas[n + 2], clip.deltas[n + 3]);
                    deltaArm = Quaternion.Slerp(deltaArm, deltaNext, frac);
                }

                // Conjugate the arm-space world delta into modelRoot-local space:
                //   D_root = R_M * D_arm * R_M^T
                Matrix4x4 dArm = Matrix4x4.Rotate(deltaArm);
                Matrix4x4 dRoot = rotArmToRoot * dArm * rotRootToArm;
                Quaternion deltaRoot = dRoot.rotation;

                bones[j].rotation = rootRot * deltaRoot * boneRestInRoot[j];
            }
        }

        void ApplyGlobal(int f, int next, float frac)
        {
            Matrix4x4 local = bindLocal;
            if (applyGlobalMotion)
            {
                var rot = new Quaternion(
                    clip.rootRot[f * 4 + 0], clip.rootRot[f * 4 + 1],
                    clip.rootRot[f * 4 + 2], clip.rootRot[f * 4 + 3]);
                var pos = new Vector3(
                    clip.rootPos[f * 3 + 0], clip.rootPos[f * 3 + 1], clip.rootPos[f * 3 + 2]);
                float s = clip.rootScale[f];
                if (frac > 0f)
                {
                    var rotN = new Quaternion(
                        clip.rootRot[next * 4 + 0], clip.rootRot[next * 4 + 1],
                        clip.rootRot[next * 4 + 2], clip.rootRot[next * 4 + 3]);
                    var posN = new Vector3(
                        clip.rootPos[next * 3 + 0], clip.rootPos[next * 3 + 1],
                        clip.rootPos[next * 3 + 2]);
                    rot = Quaternion.Slerp(rot, rotN, frac);
                    pos = Vector3.Lerp(pos, posN, frac);
                    s = Mathf.Lerp(s, clip.rootScale[next], frac);
                }

                // Global body transform lives in arm space; conjugate it into
                // modelRoot-local space, then compose onto the bind placement.
                Matrix4x4 objArm = Matrix4x4.TRS(pos, rot, new Vector3(s, s, s));
                Matrix4x4 globalRoot = armToRoot * objArm * armToRootInverse;
                local = bindLocal * globalRoot;
            }

            modelRoot.localPosition = local.GetColumn(3);
            modelRoot.localRotation = local.rotation;
            modelRoot.localScale = local.lossyScale;
        }

        /// <summary>Gaussian elimination with partial pivoting: solves A (4x4) X = B (4xm).</summary>
        static double[,] SolveLinear(double[,] a, double[,] b)
        {
            int n = 4, m = b.GetLength(1);
            var M = new double[n, n + m];
            for (int r = 0; r < n; r++)
            {
                for (int c = 0; c < n; c++) M[r, c] = a[r, c];
                for (int c = 0; c < m; c++) M[r, n + c] = b[r, c];
            }
            for (int col = 0; col < n; col++)
            {
                int pivot = col;
                for (int r = col + 1; r < n; r++)
                    if (Math.Abs(M[r, col]) > Math.Abs(M[pivot, col])) pivot = r;
                if (Math.Abs(M[pivot, col]) < 1e-12)
                    throw new InvalidOperationException("SmalMotionPlayer: calibration is singular");
                if (pivot != col)
                    for (int c = 0; c < n + m; c++)
                        (M[col, c], M[pivot, c]) = (M[pivot, c], M[col, c]);
                double d = M[col, col];
                for (int c = 0; c < n + m; c++) M[col, c] /= d;
                for (int r = 0; r < n; r++)
                {
                    if (r == col) continue;
                    double factor = M[r, col];
                    for (int c = 0; c < n + m; c++) M[r, c] -= factor * M[col, c];
                }
            }
            var x = new double[n, m];
            for (int r = 0; r < n; r++)
                for (int c = 0; c < m; c++) x[r, c] = M[r, n + c];
            return x;
        }
    }
}
