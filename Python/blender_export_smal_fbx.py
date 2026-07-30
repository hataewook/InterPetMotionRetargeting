"""Blender-side helper that builds and exports a weighted SMAL armature."""

from __future__ import annotations

import sys
from pathlib import Path

import bpy
import numpy as np


def parse_blender_args() -> tuple[Path, Path]:
    if "--" not in sys.argv:
        raise ValueError("Expected: blender ... -- <prepared.npz> <output.fbx>")
    args = sys.argv[sys.argv.index("--") + 1 :]
    if len(args) != 2:
        raise ValueError("Expected prepared NPZ and output FBX paths")
    return Path(args[0]), Path(args[1])


def reset_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)


def create_mesh(vertices: np.ndarray, faces: np.ndarray) -> bpy.types.Object:
    mesh = bpy.data.meshes.new("SMAL_dog02_neutral_mesh")
    mesh.from_pydata(vertices.tolist(), [], faces.tolist())
    mesh.update()
    mesh.validate(verbose=True)

    mesh_object = bpy.data.objects.new("SMAL_dog02_neutral", mesh)
    bpy.context.collection.objects.link(mesh_object)
    return mesh_object


def bone_tail(
    joint_index: int, joints: np.ndarray, parents: np.ndarray
) -> np.ndarray:
    children = np.flatnonzero(parents == joint_index)
    for child in children:
        direction = joints[child] - joints[joint_index]
        if np.linalg.norm(direction) > 1e-8:
            return joints[child]

    parent = int(parents[joint_index])
    if parent >= 0:
        direction = joints[joint_index] - joints[parent]
    else:
        direction = np.array([1.0, 0.0, 0.0])
    length = np.linalg.norm(direction)
    if length <= 1e-8:
        direction = np.array([1.0, 0.0, 0.0])
        length = 1.0
    return joints[joint_index] + direction / length * 0.03


def create_armature(joints: np.ndarray, parents: np.ndarray) -> bpy.types.Object:
    armature = bpy.data.armatures.new("SMAL_35_joint_armature")
    armature_object = bpy.data.objects.new("SMAL_35_joint_armature", armature)
    bpy.context.collection.objects.link(armature_object)
    armature_object.show_in_front = True

    bpy.context.view_layer.objects.active = armature_object
    armature_object.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")

    edit_bones = []
    for joint_index, joint in enumerate(joints):
        edit_bone = armature.edit_bones.new(f"SMAL_joint_{joint_index:02d}")
        edit_bone.head = joint
        edit_bone.tail = bone_tail(joint_index, joints, parents)
        edit_bones.append(edit_bone)

    for joint_index, parent in enumerate(parents):
        if parent >= 0:
            edit_bones[joint_index].parent = edit_bones[parent]
            edit_bones[joint_index].use_connect = False

    bpy.ops.object.mode_set(mode="OBJECT")
    return armature_object


def bind_skin(
    mesh_object: bpy.types.Object,
    armature_object: bpy.types.Object,
    weights: np.ndarray,
) -> None:
    for joint_index in range(weights.shape[1]):
        vertex_group = mesh_object.vertex_groups.new(
            name=f"SMAL_joint_{joint_index:02d}"
        )
        influenced = np.flatnonzero(weights[:, joint_index] > 1e-8)
        for vertex_index in influenced:
            vertex_group.add(
                [int(vertex_index)], float(weights[vertex_index, joint_index]), "REPLACE"
            )

    modifier = mesh_object.modifiers.new(name="SMAL_Armature", type="ARMATURE")
    modifier.object = armature_object
    modifier.use_vertex_groups = True
    mesh_object.parent = armature_object


def export_fbx(
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
        bake_anim=False,
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
        joints = data["joints"]
        parents = data["parents"]
        weights = data["weights"]
        betas = data["betas"]
        betas_limbs = data["betas_limbs"]

    reset_scene()
    mesh_object = create_mesh(vertices, faces)
    armature_object = create_armature(joints, parents)
    bind_skin(mesh_object, armature_object, weights)

    mesh_object["smal_model"] = "39dogs_norm_newv3"
    mesh_object["neutral_pose"] = True
    armature_object["smal_betas"] = betas.tolist()
    armature_object["smal_betas_limbs"] = betas_limbs.tolist()
    export_fbx(output_path, mesh_object, armature_object)


if __name__ == "__main__":
    main()
