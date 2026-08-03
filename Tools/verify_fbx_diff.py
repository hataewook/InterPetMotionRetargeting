#!/usr/bin/env python3
"""
FBX 두 개를 구조적으로 비교한다. Blender 라운드트립 후 스켈레톤이
290개 애니메이션과 여전히 호환되는지 검증하는 용도.

    python3 verify_fbx_diff.py 원본.fbx 재익스포트.fbx

검사 항목
  - 노드 이름 / 계층 / 타입(LimbNode·Null·Mesh)
  - 각 노드의 Lcl Translation / Rotation / Scaling  (바인드 포즈 = 애니 호환성의 핵심)
  - 스킨 클러스터별 영향 버텍스 수 및 웨이트 합
  - 메시 버텍스 / 폴리곤 / UV / 버텍스컬러 수
  - 리프본(_end) 오염 여부

의존성 없음 (표준 라이브러리만).
"""

import struct
import zlib
import sys
import collections


# ───────────────────── 바이너리 FBX 파서 ─────────────────────

class _R:
    def __init__(s, d):
        s.d, s.p = d, 0

    def u(s, f, n):
        v = struct.unpack_from(f, s.d, s.p)
        s.p += n
        return v


def _prop(r):
    t = r.d[r.p:r.p + 1].decode()
    r.p += 1
    if t == "Y": return r.u("<h", 2)[0]
    if t == "C": return bool(r.u("<B", 1)[0])
    if t == "I": return r.u("<i", 4)[0]
    if t == "F": return r.u("<f", 4)[0]
    if t == "D": return r.u("<d", 8)[0]
    if t == "L": return r.u("<q", 8)[0]
    if t in "fdlbic":
        n, enc, cl = r.u("<III", 12)
        raw = r.d[r.p:r.p + cl]; r.p += cl
        if enc == 1:
            raw = zlib.decompress(raw)
        fmt = {"f": "f", "d": "d", "l": "q", "i": "i", "b": "B", "c": "B"}[t]
        return list(struct.unpack("<%d%s" % (n, fmt), raw))
    if t in "SR":
        n = r.u("<I", 4)[0]
        raw = r.d[r.p:r.p + n]; r.p += n
        return raw.decode("utf-8", "replace") if t == "S" else raw
    raise ValueError("unknown property type %r" % t)


class Node:
    __slots__ = ("name", "props", "children")

    def __init__(s, name):
        s.name, s.props, s.children = name, [], []

    def get(s, name):
        for c in s.children:
            if c.name == name:
                return c


def _node(r, ver):
    if ver >= 7500:
        end, np_, pl = r.u("<QQQ", 24)
        nlen = r.u("<B", 1)[0]
        sentinel = 25
    else:
        end, np_, pl = r.u("<III", 12)
        nlen = r.u("<B", 1)[0]
        sentinel = 13
    if end == 0:
        return None
    name = r.d[r.p:r.p + nlen].decode("utf-8", "replace"); r.p += nlen
    n = Node(name)
    for _ in range(np_):
        n.props.append(_prop(r))
    while r.p < end - sentinel:
        c = _node(r, ver)
        if c is None:
            break
        n.children.append(c)
    r.p = end
    return n


def parse(path):
    d = open(path, "rb").read()
    if not d.startswith(b"Kaydara FBX Binary"):
        raise SystemExit(f"{path}: 바이너리 FBX가 아닙니다 (ASCII FBX는 미지원)")
    r = _R(d); r.p = 23
    ver = r.u("<I", 4)[0]
    root = Node("ROOT")
    while True:
        n = _node(r, ver)
        if n is None or r.p >= len(d) - 100:
            if n:
                root.children.append(n)
            break
        root.children.append(n)
    return root, ver


# ───────────────────── 씬 요약 추출 ─────────────────────

def summarize(path):
    root, ver = parse(path)
    D = {c.name: c for c in root.children}
    O, C = D["Objects"], D["Connections"]

    objs = {c.props[0]: c for c in O.children if c.props and isinstance(c.props[0], int)}
    nm = lambda i: objs[i].props[1].split("\x00")[0] if i in objs else "<root>"

    models = {k: v for k, v in objs.items() if v.name == "Model"}
    parent = {}
    for c in C.children:
        if c.props[0] == "OO" and c.props[1] in models:
            parent[c.props[1]] = c.props[2]

    def lcl(c):
        out = {}
        p70 = c.get("Properties70")
        if p70:
            for p in p70.children:
                if p.props and p.props[0] in ("Lcl Translation", "Lcl Rotation", "Lcl Scaling"):
                    out[p.props[0]] = tuple(round(float(x), 5) for x in p.props[4:7])
        return out

    def path_of(k):
        chain, seen = [], set()
        while k in models and k not in seen:
            seen.add(k)
            chain.append(nm(k))
            k = parent.get(k)
        return "/".join(reversed(chain))

    nodes = {}
    for k, v in models.items():
        nodes[path_of(k)] = {
            "name": nm(k),
            "type": v.props[2],
            "xform": lcl(v),
        }

    # 스킨 클러스터
    clusters = {}
    for c in C.children:
        if c.props[0] != "OO":
            continue
        s, d = c.props[1], c.props[2]
        if d in objs and objs[d].name == "Deformer" and objs[d].props[2] == "Cluster" and s in models:
            cl = objs[d]
            g = {ch.name: ch for ch in cl.children}
            idx = g["Indexes"].props[0] if "Indexes" in g else []
            w = g["Weights"].props[0] if "Weights" in g else []
            clusters[nm(s)] = (len(idx), round(sum(w), 4))

    geo = {}
    for c in O.children:
        if c.name == "Geometry":
            g = {ch.name: ch for ch in c.children}
            pvi = g["PolygonVertexIndex"].props[0] if "PolygonVertexIndex" in g else []
            uv = g.get("LayerElementUV")
            col = g.get("LayerElementColor")
            geo[c.props[1].split("\x00")[0] or "<mesh>"] = dict(
                verts=len(g["Vertices"].props[0]) // 3 if "Vertices" in g else 0,
                polys=sum(1 for i in pvi if i < 0),
                idx=len(pvi),
                uv=len(uv.get("UV").props[0]) // 2 if uv and uv.get("UV") else 0,
                color=len(col.get("Colors").props[0]) // 4 if col and col.get("Colors") else 0,
            )

    return dict(ver=ver, nodes=nodes, clusters=clusters, geo=geo)


# ───────────────────── 비교 ─────────────────────

OK, WARN, FAIL = "  ok ", " WARN", " FAIL"
_status = {"fail": 0, "warn": 0}


def report(level, msg):
    if level is FAIL:
        _status["fail"] += 1
    elif level is WARN:
        _status["warn"] += 1
    print(f"[{level}] {msg}")


def diff(a, b, pos_tol=1e-4, rot_tol=1e-3):
    print(f"\nFBX 버전: {a['ver']} -> {b['ver']}")

    na, nb = a["nodes"], b["nodes"]
    ka, kb = set(na), set(nb)

    leaf = sorted(p for p in kb - ka if p.rsplit("/", 1)[-1].endswith("_end"))
    if leaf:
        report(FAIL, f"리프본(_end) {len(leaf)}개 생성됨 — 익스포트 시 'Add Leaf Bones' 끄세요")
        for p in leaf[:5]:
            print(f"          {p}")

    added = sorted(p for p in kb - ka if not p.rsplit('/', 1)[-1].endswith('_end'))
    removed = sorted(ka - kb)

    print(f"\n── 노드: 원본 {len(ka)} / 신규 {len(kb)}")
    if added:
        report(OK, f"추가된 노드 {len(added)}개 (의도한 신규 본이면 정상)")
        for p in added:
            print(f"          + {p}  [{nb[p]['type']}]")
    if removed:
        report(FAIL, f"사라진 노드 {len(removed)}개 — 애니메이션이 깨집니다")
        for p in removed:
            print(f"          - {p}  [{na[p]['type']}]")
    if not added and not removed:
        report(OK, "노드 이름/계층 완전 일치")

    print("\n── 노드 타입")
    tchg = [(p, na[p]["type"], nb[p]["type"]) for p in sorted(ka & kb) if na[p]["type"] != nb[p]["type"]]
    if tchg:
        report(WARN, f"타입 변경 {len(tchg)}개 (Null<->LimbNode는 대개 무해)")
        for p, x, y in tchg[:20]:
            print(f"          {p}: {x} -> {y}")
    else:
        report(OK, "타입 모두 동일")

    print("\n── 바인드 포즈 (로컬 트랜스폼)")
    worst, bad = ("", 0.0, ""), 0
    for p in sorted(ka & kb):
        for key, tol in (("Lcl Translation", pos_tol), ("Lcl Rotation", rot_tol), ("Lcl Scaling", pos_tol)):
            x = na[p]["xform"].get(key, (0.0, 0.0, 0.0) if key != "Lcl Scaling" else (1.0, 1.0, 1.0))
            y = nb[p]["xform"].get(key, (0.0, 0.0, 0.0) if key != "Lcl Scaling" else (1.0, 1.0, 1.0))
            d = max(abs(u - v) for u, v in zip(x, y))
            if d > worst[1]:
                worst = (p, d, key)
            if d > tol:
                bad += 1
                if bad <= 15:
                    print(f"          {p} [{key}]\n            {x}\n            {y}   Δ={d:.6f}")
    if bad:
        report(FAIL, f"트랜스폼 불일치 {bad}건 — 애니메이션이 어긋납니다 "
                     f"(임포트 시 'Automatic Bone Orientation' 껐는지 확인)")
    else:
        report(OK, f"바인드 포즈 일치 (최대 오차 {worst[1]:.2e} @ {worst[0]} {worst[2]})")

    print("\n── 스킨 웨이트")
    ca, cb = a["clusters"], b["clusters"]
    gone = sorted(set(ca) - set(cb))
    new = sorted(set(cb) - set(ca))
    if gone:
        report(FAIL, f"스킨 클러스터 소실: {', '.join(gone)}")
    if new:
        report(OK, f"신규 스킨 본: {', '.join(new)}")
    tot_a = sum(v[1] for v in ca.values())
    tot_b = sum(v[1] for v in cb.values())
    print(f"          웨이트 총합 {tot_a:.3f} -> {tot_b:.3f}  (Δ {tot_b - tot_a:+.3f})")
    if abs(tot_b - tot_a) > 0.5:
        report(WARN, "웨이트 총합이 크게 변했습니다 — 정규화 문제 가능성")
    else:
        report(OK, "웨이트 총합 보존 (바인드 포즈에서 실루엣 동일)")

    chg = [(k, ca[k], cb[k]) for k in sorted(set(ca) & set(cb)) if ca[k] != cb[k]]
    if chg:
        print(f"          변경된 클러스터 {len(chg)}개:")
        for k, x, y in chg[:15]:
            print(f"            {k:<26} verts {x[0]:>4} -> {y[0]:<4}  sum {x[1]:.2f} -> {y[1]:.2f}")

    print("\n── 메시")
    for k in sorted(set(a["geo"]) | set(b["geo"])):
        ga, gb = a["geo"].get(k), b["geo"].get(k)
        if ga is None or gb is None:
            report(FAIL, f"메시 {k!r} 한쪽에만 존재")
            continue
        for f in ("verts", "polys", "uv", "color"):
            lvl = OK if ga[f] == gb[f] else (WARN if f in ("uv", "color") else FAIL)
            report(lvl, f"{k or '<mesh>'} {f}: {ga[f]} -> {gb[f]}")

    print("\n" + "=" * 60)
    if _status["fail"]:
        print(f"결과: FAIL {_status['fail']}건, WARN {_status['warn']}건 — 임포트/익스포트 설정을 재확인하세요.")
        return 1
    print(f"결과: 통과 (WARN {_status['warn']}건). 290개 애니메이션과 호환될 가능성이 높습니다.")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    sys.exit(diff(summarize(sys.argv[1]), summarize(sys.argv[2])))
