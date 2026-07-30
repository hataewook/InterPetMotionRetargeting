"""Blender-side helper that bakes SMAL motion onto a weighted armature."""

from __future__ import annotations

import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from blender_export_smal_fbx import bind_skin, create_armature, create_mesh, reset_scene


def parse_blender_args() -> tuple[Path, Path]:
    if "--" not in sys.argv:
        raise ValueError("Expected: blender ... -- <prepared.npz> <output.fbx>")
    args = sys.argv[sys.argv.index("--") + 1 :]
    if len(args) != 2:
        raise ValueError("Expected prepared NPZ and output FBX paths")
    return Path(args[0]), Path(args[1])


def set_linear_interpolation(armature_object: bpy.types.Object) -> None:
    action = armature_object.animation_data.action
    for curve in action.fcurves:
        for keyframe in curve.keyframe_points:
            keyframe.interpolation = "LINEAR"


def validate_baked_joint_positions(
    armature_object: bpy.types.Object,
    local_joints: np.ndarray,
    frame_indices: np.ndarray,
) -> None:
    sample_indices = [0, len(frame_indices) // 2, len(frame_indices) - 1]
    errors = []
    for sample_index in sample_indices:
        bpy.context.scene.frame_set(int(frame_indices[sample_index]) - 1)
        bpy.context.view_layer.update()
        for joint_index in range(local_joints.shape[1]):
            actual = armature_object.pose.bones[
                f"SMAL_joint_{joint_index:02d}"
            ].head
            errors.append(
                (actual - Vector(local_joints[sample_index, joint_index])).length
            )
    max_error = max(errors)
    print(f"Pre-export joint validation: mean={sum(errors) / len(errors):.8g}, max={max_error:.8g}")
    if max_error > 1e-4:
        raise RuntimeError(f"Baked joint position error is too large: {max_error}")


def bake_animation(
    armature_object: bpy.types.Object,
    local_joints: np.ndarray,
    rotation_deltas: np.ndarray,
    world_rotations: np.ndarray,
    world_translations: np.ndarray,
    world_scales: np.ndarray,
    frame_indices: np.ndarray,
) -> None:
    scene = bpy.context.scene
    scene.render.fps = 60
    scene.render.fps_base = 1.0
    timeline_frames = frame_indices.astype(np.int64) - 1
    scene.frame_start = int(timeline_frames[0])
    scene.frame_end = int(timeline_frames[-1])

    armature_object.rotation_mode = "QUATERNION"
    rest_matrices = [
        armature_object.data.bones[f"SMAL_joint_{index:02d}"].matrix_local.copy()
        for index in range(local_joints.shape[1])
    ]

    for sample_index, timeline_frame in enumerate(timeline_frames):
        frame = int(timeline_frame)
        scene.frame_set(frame)

        armature_object.location = Vector(world_translations[sample_index])
        armature_object.rotation_quaternion = Matrix(
            world_rotations[sample_index]
        ).to_quaternion()
        scale = float(world_scales[sample_index])
        armature_object.scale = (scale, scale, scale)
        armature_object.keyframe_insert("location", frame=frame)
        armature_object.keyframe_insert("rotation_quaternion", frame=frame)
        armature_object.keyframe_insert("scale", frame=frame)

        desired_matrices = []
        for joint_index in range(local_joints.shape[1]):
            desired_rotation = (
                Matrix(rotation_deltas[sample_index, joint_index])
                @ rest_matrices[joint_index].to_3x3()
            )
            desired_matrix = desired_rotation.to_4x4()
            desired_matrix.translation = Vector(
                local_joints[sample_index, joint_index]
            )
            desired_matrices.append(desired_matrix)

        for joint_index in range(local_joints.shape[1]):
            pose_bone = armature_object.pose.bones[f"SMAL_joint_{joint_index:02d}"]
            pose_bone.rotation_mode = "QUATERNION"
            bone = pose_bone.bone
            if pose_bone.parent is None:
                matrix_basis = bone.convert_local_to_pose(
                    desired_matrices[joint_index],
                    bone.matrix_local,
                    invert=True,
                )
            else:
                parent_index = int(pose_bone.parent.name.rsplit("_", 1)[1])
                matrix_basis = bone.convert_local_to_pose(
                    desired_matrices[joint_index],
                    bone.matrix_local,
                    parent_matrix=desired_matrices[parent_index],
                    parent_matrix_local=pose_bone.parent.bone.matrix_local,
                    invert=True,
                )
            pose_bone.matrix_basis = matrix_basis
            pose_bone.keyframe_insert("location", frame=frame)
            pose_bone.keyframe_insert("rotation_quaternion", frame=frame)
            pose_bone.keyframe_insert("scale", frame=frame)

    action = armature_object.animation_data.action
    action.name = "InterPet_dog02_p04_take01_ego_001"
    set_linear_interpolation(armature_object)
    validate_baked_joint_positions(armature_object, local_joints, frame_indices)
    scene.frame_set(int(timeline_frames[0]))


def export_animated_fbx(
    output_path: Path,
    mesh_object: bpy.types.Object,
    armature_object: bpy.types.Object,
) -> None:
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0
    bpy.ops.object.select_all(action="DESELECT")
    mesh_object.select_set(True)
    armature_object.select_set(True)
    bpy.context.view_layer.objects.active = armature_object

    output_path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.fbx(
        filepath=str(output_path),
        use_selection=True,
        object_types={"ARMATURE", "MESH"},
        use_mesh_modifiers=False,
        add_leaf_bones=False,
        bake_anim=True,
        bake_anim_use_all_bones=True,
        bake_anim_use_nla_strips=False,
        bake_anim_use_all_actions=False,
        bake_anim_force_startend_keying=True,
        bake_anim_step=1.0,
        bake_anim_simplify_factor=0.0,
        axis_forward="-Z",
        axis_up="Y",
        primary_bone_axis="Y",
        secondary_bone_axis="X",
        use_custom_props=True,
    )


def main() -> None:
    prepared_path, output_path = parse_blender_args()
    with np.load(prepared_path) as data:
        vertices = data["vertices"]
        faces = data["faces"]
        rest_joints = data["rest_joints"]
        parents = data["parents"]
        weights = data["weights"]
        local_joints = data["local_joints"]
        rotation_deltas = data["rotation_deltas"]
        world_rotations = data["world_rotations"]
        world_translations = data["world_translations"]
        world_scales = data["world_scales"]
        frame_indices = data["frame_indices"]
        betas = data["betas"]
        betas_limbs = data["betas_limbs"]

    reset_scene()
    mesh_object = create_mesh(vertices, faces)
    armature_object = create_armature(rest_joints, parents)
    bind_skin(mesh_object, armature_object, weights)
    bake_animation(
        armature_object,
        local_joints,
        rotation_deltas,
        world_rotations,
        world_translations,
        world_scales,
        frame_indices,
    )

    mesh_object["smal_model"] = "39dogs_norm_newv3"
    mesh_object["neutral_pose_bind"] = True
    armature_object["smal_betas"] = betas.tolist()
    armature_object["smal_betas_limbs"] = betas_limbs.tolist()
    armature_object["source_fps"] = 60
    export_animated_fbx(output_path, mesh_object, armature_object)


if __name__ == "__main__":
    main()
