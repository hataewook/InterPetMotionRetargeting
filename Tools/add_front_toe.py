"""
BullTerrier 앞발에 발가락 관절 1마디 추가 (Blender 3.6 / 4.x)

기존:  Bip001 L Hand -> Bip001 L Finger0 (말단, 스킨 웨이트 47)
변경:  Bip001 L Hand -> Bip001 L Finger0 -> Bip001 L Finger01 (신규)

Finger0의 웨이트 중 발끝 쪽 부분을 새 본으로 스무스하게 이전한다.
웨이트 합은 보존되므로 별도 정규화 불필요 = 바인드 포즈에서 메시가 전혀 변하지 않음.

사용법
------
1) Blender에서 BullTerrier.fbx를 아래 IMPORT 설정으로 임포트
2) Scripting 탭 > 이 파일 열기 > Run Script
3) 콘솔(Window > Toggle System Console) 진단 출력 확인
4) 아래 EXPORT 설정으로 재익스포트

헤드리스 실행:
    blender -b -P add_front_toe.py
    (RUN_HEADLESS = True 로 바꾸면 임포트/익스포트까지 자동)
"""

import bpy
import bmesh  # noqa: F401
from mathutils import Vector

# ─────────────────────────── 설정 ───────────────────────────

SIDES = ["L", "R"]

PARENT_FMT = "Bip001 {side} Finger0"    # 기존 앞발 발가락 본
GRANDPA_FMT = "Bip001 {side} Hand"      # 그 부모 (축 폴백용)
NEW_FMT = "Bip001 {side} Finger01"      # 신규 본 (3ds Max Biped 명명 규칙)

# 새 관절을 Finger0 웨이트 영역의 어디에 놓을지 (0=시작, 1=발끝)
SPLIT_RATIO = 0.55
# 웨이트가 섞이는 구간 폭 (웨이트 영역 길이 대비 비율). 0이면 칼같이 잘림
BLEND_RATIO = 0.30
# 새 본 길이 (웨이트 영역 길이 대비). 시각적 표시용, 디폼에는 영향 없음
TAIL_RATIO = 0.45
# 새 본이 최대로 가져갈 수 있는 웨이트 비율. 1.0이면 발끝은 100% 새 본이 지배
MAX_TAKEOVER = 1.0

# 뒷발 Toe0(현재 Null/웨이트 0)도 같이 실제 본으로 승격하려면 True
ALSO_FIX_REAR_TOES = False

RUN_HEADLESS = False
FBX_IN = "/Users/htw/workplace/PetDemo/Assets/Dog_Bullterrier/Base/BullTerrier.fbx"
FBX_OUT = "/Users/htw/workplace/PetDemo/Assets/Dog_Bullterrier/Base/BullTerrier_toe.fbx"


# ───────────────────────── 유틸 ─────────────────────────

def log(*a):
    print("[toe]", *a)


def smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def find_rig():
    """Finger0 본을 가진 아머처와, 그 아머처로 디폼되는 메시를 찾는다."""
    probe = PARENT_FMT.format(side=SIDES[0])
    arm = None
    for ob in bpy.data.objects:
        if ob.type == "ARMATURE" and probe in ob.data.bones:
            arm = ob
            break
    if arm is None:
        raise RuntimeError(f"'{probe}' 본을 가진 아머처를 찾을 수 없습니다. FBX를 먼저 임포트하세요.")

    mesh = None
    for ob in bpy.data.objects:
        if ob.type != "MESH":
            continue
        for m in ob.modifiers:
            if m.type == "ARMATURE" and m.object == arm:
                mesh = ob
                break
        if mesh:
            break
    if mesh is None:
        raise RuntimeError("해당 아머처를 쓰는 스킨드 메시를 찾을 수 없습니다.")
    return arm, mesh


def group_verts(mesh, gname):
    """버텍스 그룹에 속한 (vertex_index, weight) 목록."""
    vg = mesh.vertex_groups.get(gname)
    if vg is None:
        return None, []
    gi = vg.index
    out = []
    for v in mesh.data.vertices:
        for g in v.groups:
            if g.group == gi:
                out.append((v.index, g.weight))
                break
    return vg, out


# ─────────────────── 1단계: 축 / 분할 위치 계산 ───────────────────

def measure(arm, mesh, side):
    pname = PARENT_FMT.format(side=side)
    gname = GRANDPA_FMT.format(side=side)

    bone = arm.data.bones.get(pname)
    if bone is None:
        raise RuntimeError(f"본 없음: {pname}")

    head_w = arm.matrix_world @ bone.head_local

    vg, pairs = group_verts(mesh, pname)
    if not pairs:
        raise RuntimeError(f"버텍스 그룹 '{pname}'에 가중치가 없습니다.")

    mw = mesh.matrix_world
    pts = [(vi, w, mw @ mesh.data.vertices[vi].co) for vi, w in pairs]

    # 발 축: 웨이트 영역 중심 방향 (기하학적으로 가장 안정적)
    total = sum(w for _, w, _ in pts)
    centroid = Vector((0, 0, 0))
    for _, w, p in pts:
        centroid += p * w
    centroid /= total
    axis = centroid - head_w

    if axis.length < 1e-6:
        # 폴백: 상위 본 방향
        gb = arm.data.bones.get(gname)
        axis = head_w - (arm.matrix_world @ gb.head_local)
    axis.normalize()

    ts = [(p - head_w).dot(axis) for _, _, p in pts]
    t_min, t_max = min(ts), max(ts)
    span = t_max - t_min
    if span < 1e-6:
        raise RuntimeError(f"{pname}: 웨이트 영역이 너무 좁습니다 (span={span}).")

    split = t_min + SPLIT_RATIO * span
    blend = max(BLEND_RATIO * span, 1e-6)

    log(f"{pname}: verts={len(pts)} axis={tuple(round(c,3) for c in axis)} "
        f"t={t_min:.4f}..{t_max:.4f} (span {span:.4f}) split={split:.4f} blend={blend:.4f}")

    return dict(side=side, parent=pname, bone=bone, head_w=head_w, axis=axis,
                pts=pts, t_min=t_min, t_max=t_max, span=span,
                split=split, blend=blend, vg=vg)


# ─────────────────── 2단계: 본 생성 ───────────────────

def create_bones(arm, plans):
    bpy.context.view_layer.objects.active = arm
    prev = arm.mode
    bpy.ops.object.mode_set(mode="EDIT")
    eb = arm.data.edit_bones
    inv = arm.matrix_world.inverted()

    created = []
    for pl in plans:
        new_name = NEW_FMT.format(side=pl["side"])
        if new_name in eb:
            log(f"이미 존재 -> 재사용: {new_name}")
            nb = eb[new_name]
        else:
            nb = eb.new(new_name)
        parent_eb = eb[pl["parent"]]

        head_w = pl["head_w"] + pl["axis"] * pl["split"]
        tail_w = head_w + pl["axis"] * (pl["span"] * TAIL_RATIO)

        nb.head = inv @ head_w
        nb.tail = inv @ tail_w
        nb.parent = parent_eb
        nb.use_connect = False
        nb.use_deform = True
        try:
            nb.align_roll(parent_eb.z_axis)
        except Exception:
            nb.roll = parent_eb.roll

        # 부모가 말단이라 임의 길이를 갖고 있으면, 새 본 head까지로 정리
        if len(parent_eb.children) == 1:
            parent_eb.tail = nb.head

        created.append(new_name)
        log(f"본 생성: {new_name}  parent={pl['parent']}  "
            f"head={tuple(round(c,4) for c in nb.head)} tail={tuple(round(c,4) for c in nb.tail)}")

    bpy.ops.object.mode_set(mode="OBJECT")
    if prev != "OBJECT":
        try:
            bpy.ops.object.mode_set(mode=prev)
        except Exception:
            pass
    return created


# ─────────────────── 3단계: 웨이트 이전 ───────────────────

def transfer_weights(mesh, plans):
    for pl in plans:
        new_name = NEW_FMT.format(side=pl["side"])
        src = mesh.vertex_groups.get(pl["parent"])
        dst = mesh.vertex_groups.get(new_name) or mesh.vertex_groups.new(name=new_name)

        moved = 0
        moved_w = 0.0
        for vi, w, p in pl["pts"]:
            t = (p - pl["head_w"]).dot(pl["axis"])
            f = smoothstep((t - pl["split"]) / pl["blend"] + 0.5) * MAX_TAKEOVER
            if f <= 1e-5:
                continue
            take = w * f
            dst.add([vi], take, "REPLACE")
            src.add([vi], w - take, "REPLACE")
            moved += 1
            moved_w += take

        log(f"{new_name}: {moved}/{len(pl['pts'])} verts, 이전된 웨이트 총합 {moved_w:.3f}")


# ─────────────────── 뒷발 Toe0 승격 (선택) ───────────────────

def fix_rear_toes(arm, mesh):
    """뒷발 Toe0은 현재 웨이트 0인 마커. Foot 웨이트 중 발끝을 나눠준다."""
    for side in SIDES:
        toe = f"Bip001 {side} Toe0"
        foot = f"Bip001 {side} Foot"
        if toe not in arm.data.bones:
            log(f"건너뜀 (본 없음): {toe}")
            continue
        fb = arm.data.bones[foot]
        tb = arm.data.bones[toe]
        head_w = arm.matrix_world @ fb.head_local
        axis = (arm.matrix_world @ tb.head_local) - head_w
        if axis.length < 1e-6:
            continue
        axis.normalize()

        _, pairs = group_verts(mesh, foot)
        if not pairs:
            continue
        mw = mesh.matrix_world
        pts = [(vi, w, mw @ mesh.data.vertices[vi].co) for vi, w in pairs]
        ts = [(p - head_w).dot(axis) for _, _, p in pts]
        t_min, t_max = min(ts), max(ts)
        span = t_max - t_min
        split = t_min + 0.65 * span
        blend = 0.30 * span

        src = mesh.vertex_groups[foot]
        dst = mesh.vertex_groups.get(toe) or mesh.vertex_groups.new(name=toe)
        moved = 0
        for (vi, w, p), t in zip(pts, ts):
            f = smoothstep((t - split) / blend + 0.5)
            if f <= 1e-5:
                continue
            dst.add([vi], w * f, "REPLACE")
            src.add([vi], w * (1 - f), "REPLACE")
            moved += 1
        tb.use_deform = True
        log(f"{toe}: {moved} verts 승격")


# ─────────────────── 임포트 / 익스포트 ───────────────────

IMPORT_KW = dict(
    global_scale=1.0,
    use_custom_normals=True,
    use_anim=False,
    automatic_bone_orientation=False,   # 원본 본 행렬 보존 — 절대 True 금지
    ignore_leaf_bones=False,
    force_connect_children=False,
)

EXPORT_KW = dict(
    apply_unit_scale=True,
    apply_scale_options="FBX_SCALE_NONE",
    global_scale=1.0,
    axis_forward="-Z",
    axis_up="Y",
    object_types={"EMPTY", "ARMATURE", "MESH"},
    use_mesh_modifiers=False,           # 아머처 모디파이어 적용 금지
    mesh_smooth_type="FACE",
    colors_type="SRGB",                 # 원본 버텍스 컬러 보존
    use_tspace=False,
    add_leaf_bones=False,               # _end 더미 본 생성 금지
    use_armature_deform_only=False,     # 웨이트 없는 마커 본/Empty 유지
    primary_bone_axis="Y",
    secondary_bone_axis="X",
    armature_nodetype="NULL",
    bake_anim=False,
    path_mode="AUTO",
)


def do_import():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=FBX_IN, **IMPORT_KW)
    log(f"임포트 완료: {FBX_IN}")


def do_export():
    kw = dict(EXPORT_KW)
    # Blender 버전별로 없는 인자 제거
    valid = set(bpy.ops.export_scene.fbx.get_rna_type().properties.keys())
    for k in list(kw):
        if k not in valid:
            log(f"익스포트 옵션 미지원(무시): {k}")
            kw.pop(k)
    bpy.ops.export_scene.fbx(filepath=FBX_OUT, use_selection=False, **kw)
    log(f"익스포트 완료: {FBX_OUT}")


# ─────────────────── 메인 ───────────────────

def main():
    if RUN_HEADLESS:
        do_import()

    arm, mesh = find_rig()
    log(f"아머처={arm.name}  메시={mesh.name}  본 {len(arm.data.bones)}개  "
        f"버텍스그룹 {len(mesh.vertex_groups)}개")

    plans = [measure(arm, mesh, s) for s in SIDES]
    create_bones(arm, plans)
    transfer_weights(mesh, plans)

    if ALSO_FIX_REAR_TOES:
        fix_rear_toes(arm, mesh)

    log(f"완료. 본 {len(arm.data.bones)}개 / 버텍스그룹 {len(mesh.vertex_groups)}개")

    if RUN_HEADLESS:
        do_export()


if __name__ == "__main__":
    main()
