# Foot Grounding (raycast) 최종 스테이지 — 수정 계획서

> 방식 B: collider를 "높이 질의"로만 쓰고, 관절은 기존 IK로 꺾는다.
> 물리(active ragdoll)는 쓰지 않음. 순수 kinematic 후처리 스테이지.

---

## 1. 목표 / 범위

- 모든 solve가 끝난 뒤 **최종 스테이지**에서 각 발을 바닥 collider에 레이캐스트해 접지 높이를 얻는다.
- 바닥을 **뚫는 발만** 위로 올린다(penetration clamp). 공중에 든(스윙) 발은 건드리지 않는다 → 별도 접지판정(FootPlanter) 불필요.
- 접지 보정에 맞춰 **골반/척추가 자연스럽게 반응**한다(루트 수직 이동 + 피치 재분배 = Level 2).
- `SmalRetargeter`, `NormalizedDogPoseDriver`(및 부모 `DogPoseDriver`) **양쪽 공통** 로직.
- 평면 바닥 우선, 경사/계단은 레이캐스트라 자동 대응(추가 튜닝만).

---

## 2. 아키텍처 결정

### 2.1 실행 순서 (핵심)

grounding은 **RootMotionAnchor 뒤**, 즉 최종 world 공간에서 돌아야 한다. 그래야 바닥 collider와 실제 최종 위치로 레이캐스트가 맞는다.

```
Driver pose            (order 0 / 100)   ← RestoreBind + 기존 solve
RootMotionAnchor       (order 1000)      ← yaw + XZ 오프셋만
FootGroundingStage     (order 2000)      ← 레이캐스트 + 골반 수직/피치 + 발 IK  ★신규
```

### 2.2 앵커와의 역할 분담 (중요)

grounding이 수직을 전담하므로, RootMotionAnchor의 `matchAnchorHeight`는 **끈다**(grounding과 충돌 방지). 역할을 이렇게 나눈다:

- **RootMotionAnchor** = "어디서/어느 방향" (XZ 위치 + yaw)
- **FootGrounding** = "지면 접촉" (Y + 다리 접힘/골반 반응)

`RootMotionAnchor.matchAnchorHeight`가 켜져 있으면 grounding이 무의미한 수직 오프셋 위에서 도니, grounding 사용 시 인스펙터에서 off 하도록 툴팁/문서에 명시.

### 2.3 컴포넌트 구성

다리 본 배열·pole·bendAxis·roll 파라미터가 드라이버 **private**이라, "완전 외부 컴포넌트가 알아서" 방식은 배관이 과함. 절충안:

- **공유 순수 로직**: `static class FootGrounding` — 레이캐스트 + per-foot target 계산 + 골반 재분배 + per-foot TwoBoneIK 재solve. 리그 무관 순수 함수.
- **드라이버 공개 메서드**: 각 드라이버가 `IGroundable.ApplyFootGrounding(FootGroundingSettings)` 구현 — 자기 다리 데이터를 모아 `FootGrounding`에 넘기고, grounding 뒤 의존 포즈(발가락 FK 등)를 재적용.
- **얇은 스테이지 컴포넌트**: `FootGroundingStage : MonoBehaviour` (`[DefaultExecutionOrder(2000)]`) — 인스펙터 설정을 들고, `LateUpdate`에서 `GetComponent<IGroundable>().ApplyFootGrounding(settings)` 호출. 순서 보장 + 옵션 on/off + RootMotionAnchor 없이도 동작.

> 대안: RootMotionAnchor를 안 쓰는 경우엔 드라이버 자체에 토글을 두고 LateUpdate 끝에서 직접 호출해도 됨. 하지만 앵커를 쓰면 순서 때문에 별도 스테이지가 깔끔.

---

## 3. 파일 변경 목록

**신규**
- `Assets/Scripts/FootGrounding.cs` — `IGroundable` 인터페이스, `FootGroundingSettings`(serializable), `GroundLeg` 구조체, `static class FootGrounding` 로직, `FootGroundingStage` 컴포넌트.

**수정**
- `Assets/Scripts/SmalRetargeter.cs`
  - `: IGroundable` 추가 (이미 `IBodyFrameProvider` 구현 중 → `BodyRoot` 재사용).
  - `public void ApplyFootGrounding(FootGroundingSettings s)` 추가: `legs`/`legUpper`/`legLower`/`legTip`/`legBendAxis`/`legLenTarget`, roll-lock 참조(`bindLocalUpper/Lower`,`rollAxisUpper/Lower`,`maxLegRollDegrees`), front/hind 구분으로 `GroundLeg[]` 구성 → `FootGrounding.Solve` 호출 → **발가락 FK 루프 재실행**(LateUpdate 676–684행 로직을 메서드로 추출해 재사용).
  - pole 재사용을 위해 마지막 프레임의 leg pole/target을 필드에 캐시(현재는 LateUpdate 지역변수).
- `Assets/Scripts/DogPoseDriver.cs`
  - `: IGroundable` 추가 (`BodyRoot` 재사용).
  - `public void ApplyFootGrounding(FootGroundingSettings s)` 추가: `legs`(ResolvedLeg)로 `GroundLeg[]` 구성(0,1=front / 2,3=hind), pole은 `GetLegIKTarget`/`PoleWeight` 캐시 재사용, roll-lock(`LimitRoll`) 재사용 → `FootGrounding.Solve` 호출. 발가락 FK 없음(머리 aim은 독립) → 후처리 불필요.
  - 마지막 프레임 pole 캐시 필드 추가.
- (선택) `Assets/Scripts/RootMotionAnchor.cs` — `matchAnchorHeight` 툴팁에 "grounding 사용 시 off" 문구 추가.

`FootGrounding.cs`용 `.meta`는 Unity가 자동 생성.

---

## 4. 데이터 구조 / 인터페이스

```csharp
public interface IGroundable
{
    Transform BodyRoot { get; }                 // IBodyFrameProvider와 동일 소스
    void ApplyFootGrounding(FootGroundingSettings settings);
}

// 한 발의 grounding에 필요한 최소 참조 (드라이버가 채워 넘김)
public struct GroundLeg
{
    public Transform upper, lower, tip;
    public Vector3 pole;        // 직전 solve의 pole (재현성)
    public bool isFront;        // 골반 재분배 그룹핑
    public float softZone;      // ikSoftZone
    // roll-lock (옵션): 델리게이트로 넘겨 드라이버 구현 재사용
    public System.Action postSolve;   // 예: LimitRoll 두 번 호출 (없으면 null)
}

[System.Serializable]
public struct FootGroundingSettings
{
    public bool enabled;
    public LayerMask groundMask;
    public float castUp;          // 발 위 레이 시작 높이 (m), 예 0.5
    public float maxDistance;     // 레이 최대 길이 (m), 예 1.5
    public float footOffset;      // 접촉점에서 tip까지 오프셋(발 반경), 예 0.02

    [Range(0f,1f)] public float bodyFollow;   // 공통 성분을 몸이 따라가는 비율(기본 1)
    [Range(0f,1f)] public float pitchFollow;  // 앞뒤 차이를 피치로 흡수하는 비율(기본 0.5)
    public float maxPitchDegrees;             // 피치 클램프(기본 12)

    public bool allowPullDownWhenPlanted;     // 떠 있는 접지 발도 당길지(기본 false)
    public float snapBand;                     // 위 옵션의 허용 밴드(m)

    public float heightDamp;      // groundY/골반오프셋 시간 평활(초), 예 0.06
    public bool drawGizmos;
}
```

up축: `FootGroundingStage`가 같은 오브젝트의 `RootMotionAnchor`에서 anchor up을 읽고(없으면 `Vector3.up`) `FootGrounding.Solve`에 넘긴다.

---

## 5. 알고리즘 (FootGrounding.Solve, 프레임마다)

입력: `Transform root`, `GroundLeg[] legs`, `Vector3 up`, `FootGroundingSettings s`, 그리고 상태 캐시(평활용, 스테이지가 보관).

**Step 1 — 발별 지면 질의**
- `origin = tip.position + up * s.castUp`
- `Physics.Raycast(origin, -up, out hit, s.maxDistance, s.groundMask)`
- hit → `contactAlongUp = Dot(hit.point, up)`, `targetY_i = contactAlongUp + s.footOffset`
- no hit → 그 발은 grounding 비활성(active=false), 현재 포즈 유지.

**Step 2 — 관통 clamp (스윙 보존)**
- `h_i = Dot(tip.position, up)`  (현재 발 높이)
- `desiredY_i = max(h_i, targetY_i)`  → 뚫는 발만 위로.
- `allowPullDownWhenPlanted` && `(h_i - targetY_i) in (0, snapBand]` 이면 `desiredY_i = targetY_i`(살짝 떠 있는 접지 발을 지면으로 스냅).
- `corr_i = desiredY_i - h_i`  (≥ 0, 스냅 옵션 시 음수 가능)

**Step 3 — 골반/척추 재분배 (Level 2, 리지드)**
- 활성 발을 front/hind로 나눠 `frontRise = max(corr front)`, `hindRise = max(corr hind)`.
- **공통 성분(수직)**: `v = min(frontRise, hindRise) * s.bodyFollow`
  - 근거: 네 발이 다 같은 깊이로 지면 아래면(캡처 통째로 아래) 다리를 접는 게 아니라 **몸 전체를 v만큼 올려야** 맞다. 그래서 `bodyFollow` 기본 1.
- **차이 성분(피치)**: `theta = atan2((frontRise - hindRise) * s.pitchFollow, wheelbase)`, `maxPitchDegrees`로 클램프.
  - `wheelbase = |frontGirdleMid - hindGirdleMid|` (어깨/힙 중점 거리; 드라이버의 `TryGetBodyFrame` 소스 재사용 or tip들로 근사).
  - 피벗 = 몸 중심, 축 = lateral(좌우) = `Cross(up, bodyForward)`.
- root에 리지드 적용: `root.position += up * v; root.RotateAround(pivot, lateralAxis, theta)`.
  - 리지드라 척추·엉덩이·머리가 함께 자연스럽게 기움(= 원하는 "자연스러운 반응").
  - **주의**: 피치는 머리까지 기울인다. 과하면 어색 → `maxPitchDegrees` 작게(≈8~12), `pitchFollow`로 조절. 피치 0으로 두면 순수 수직-follow + 발 IK만(더 안전).
- 골반 이동 후 발 world 위치가 바뀌었으므로 `h_i`, `corr_i` **재계산**(남은 보정만 IK가 처리).

**Step 4 — 발별 IK (수평 고정, 높이만)**
- `targetPos = tip.position + up * (desiredY_i - Dot(tip.position, up))`  ← XZ 그대로, Y만 desired.
- `TwoBoneIK.Solve(upper, lower, tip, targetPos, pole, softZone[, ref bendAxis])`
  - SmalRetargeter는 `ref bendAxis` 오버로드, DogPoseDriver는 non-ref 오버로드.
- `leg.postSolve?.Invoke()`  ← roll-lock(LimitRoll/LockRoll) 재적용.
- 발 올리기는 다리 접힘이라 항상 도달 가능. `desiredY`가 힙보다 높아 과굴곡 위험 시 clamp(다리 rest 길이의 일정 비율 이내).

**Step 5 — 의존 포즈 재적용 (드라이버 측 ApplyFootGrounding에서)**
- SmalRetargeter: 발가락 FK 루프 재실행(하퇴가 움직였으므로 toe가 stale). 현재 `LateUpdate` 676–684행을 `SolveToeFk()`로 추출해 grounding 뒤 호출.
- DogPoseDriver: 없음.

**Step 6 — 시간 평활**
- `targetY_i`, 골반 `v`/`theta`를 `heightDamp`로 `MoveTowards`/지수감쇠 → 레이가 엣지/다른 면을 스칠 때 튐 방지. 상태는 `FootGroundingStage`가 발 index별로 보관.

---

## 6. 두 드라이버 통합 지점 (구체)

### SmalRetargeter
- 다리 pole/target을 필드로 캐시: `LateUpdate`의 IK 블록(648–668행)에서 계산한 `pole`을 `lastPole[i]`, bendAxis는 이미 `legBendAxis[i]` 필드.
- `ApplyFootGrounding`:
  1. `GroundLeg[]` 구성: front = `legs[i].name`에 "Front"/"F_" 포함 여부 or 인덱스 규약(맵 정의 확인 필요). `isFront` 판정 규칙을 `SmalRetargetMap`에 상수로.
  2. `postSolve = () => { LockRoll(upper,...); LockRoll(lower,...); }`.
  3. `FootGrounding.Solve(targetRoot, legs, up, s, cache)`.
  4. `SolveToeFk()` 재실행.
- 발가락 FK 추출: 676–684행 → `void SolveToeFk(Quaternion invSRot, Quaternion tRot)` (tRot는 grounding으로 몸이 기울었으니 **재측정** 필요 → 골반 피치 후 `TorsoFrame` 다시).

### DogPoseDriver
- `SolveLegs`에서 `pole`(784–785행, PoleWeight 반영 후)을 `lastPole[index]`에 캐시.
- `ApplyFootGrounding`:
  1. `GroundLeg[]`: index 0,1 = front / 2,3 = hind (LegKeypoints 순서 확정됨).
  2. `postSolve = () => { if(limitIkLegRoll){ LimitRoll(upper..); LimitRoll(lower..);} }`.
  3. `FootGrounding.Solve(dogRoot, legs, up, s, cache)`.
- 후처리 없음.

---

## 7. 인스펙터 (FootGroundingStage)

`FootGroundingSettings` 그대로 노출 + `enabled`. 같은 오브젝트에 `IGroundable`(드라이버)와 (선택) `RootMotionAnchor` 존재 가정. `Awake`에서 `GetComponent<IGroundable>()` 필수 검증.

---

## 8. 엣지 케이스 / 리스크

- **스윙 발이 지면 위**: clamp가 안 당기므로 자연 보존. ✔
- **떠 있는 접지 발**: 기본은 방치. 필요 시 `allowPullDownWhenPlanted` + `snapBand`로만 소량 당김(과하면 발 흡착처럼 보임).
- **피치가 머리까지 기울임**: `maxPitchDegrees` 작게, 최악엔 `pitchFollow=0`(수직만).
- **RootMotionAnchor와 이중 수직**: grounding 사용 시 `matchAnchorHeight=off` 필수(문서화).
- **레이 시작점이 지면 아래**: `castUp`을 충분히(발이 이미 깊이 뚫렸을 때도 위에서 시작하도록) 크게.
- **과굴곡**: `desiredY`를 다리 rest 길이 비율로 clamp.
- **평활 지연 vs 반응성**: `heightDamp` 튜닝. 빠른 지형 변화(계단)에선 낮게.
- **여러 collider/레이어**: `groundMask`로 바닥만.
- **골반 RotateAround 후 tRot 재측정**: 발가락/후속 FK가 몸 기울기를 반영하도록 grounding 뒤 torso frame 재계산.

---

## 9. 단계적 구현 (검증 쉬운 순서)

- **Phase 1 — Level 1 (발 IK만)**: Step 1,2,4,5,6. 골반 재분배(Step 3) 생략(`bodyFollow=0,pitchFollow=0`). 발이 바닥에 붙고 스윙 보존되는지부터 확인.
- **Phase 2 — Level 2 (골반 반응)**: Step 3 추가. `bodyFollow` 1, `pitchFollow` 점진 상향하며 자연스러움 튜닝.
- (선택) **Phase 3 — Level 3**: 척추 가산 벤딩(거들별 분리). 별도 계획 필요, 지금 범위 밖.

---

## 10. 테스트 / 검증

- 평면 collider 위에서: 네 발이 한 높이로 붙는지, 걷기 시 스윙 발이 안 눌리는지.
- 경사/계단 collider: 발이 지형을 따라가고 몸이 피치로 자연 반응하는지.
- RootMotionAnchor 병행: 시작 위치/heading + 접지가 동시에 맞는지, 이중 수직 없는지.
- `enabled=false`면 기존 동작과 100% 동일한지(회귀).
- Gizmo(Step: 레이/접촉점/targetY/골반 오프셋) On으로 시각 확인.

---

## 11. 기즈모 / 디버그

`FootGroundingStage.OnDrawGizmosSelected`: 발별 레이(노랑), 접촉점(초록), desiredY 평면 표시, 골반 이동 화살표(시안). `drawGizmos` 토글.

---

### 요약 한 줄
> 신규 `FootGrounding.cs`(공유 로직 + `IGroundable` + `FootGroundingStage@order 2000`)를 만들고, 두 드라이버에 `ApplyFootGrounding`만 얇게 붙인다. 레이캐스트로 발 높이를 얻어 관통만 clamp(스윙 보존), 공통 성분은 골반 수직 이동·차이는 피치로 흡수한 뒤 남은 건 발 IK. 앵커는 XZ+yaw만 맡고 수직은 grounding이 전담.
