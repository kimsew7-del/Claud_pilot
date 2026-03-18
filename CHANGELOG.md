# Claud_pilot 변경점 (OPKR_C2 기반)

## 개요
- **베이스**: openpilotkr/openpilot OPKR_C2 브랜치
- **대상 차량**: 현대 그랜저 IG 하이브리드, 기아 K7 하이브리드
- **대상 디바이스**: comma EON (배터리리스)
- **수정 규모**: 14개 파일, +430줄 / -120줄

---

## 1. 차량 파라미터 최적화 (interface.py, tunes.py)

### 그랜저 IG HEV
- steerRatio: 14.5 (master AZERA 기준값으로 업데이트)
- steerActuatorDelay: 0.25 → 0.1초 (응답성 개선)
- longitudinalActuatorDelay: 1.0 → 0.5초

### K7 HEV YG
- steerRatio: 16.8 (master KIA_K7 기준값으로 업데이트)
- steerActuatorDelay: 0.25 → 0.1초
- longitudinalActuatorDelay: 1.0 → 0.5초

### HEV 공통 정지/출발 파라미터
- vEgoStopping: 0.8 → 0.3 m/s (회생제동으로 낮은 속도까지 제동 가능)
- vEgoStarting: 0.8 → 0.3 m/s
- stopAccel: -2.0 → -1.5 m/s² (부드러운 정지)
- stoppingDecelRate: 1.0 → 0.6 (점진적 감속)

### TORQUE_HEV_SEDAN lateral 프리셋 (신규)
- 대형 HEV 세단(그랜저/K7) 전용 횡방향 튜닝
- maxLatAccel × 0.85: 무거운 차량의 관성 보상
- TorqueFriction × 1.3: 무거운 EPS 특성 반영
- steeringAngleDeadzone ≥ 1.0°: 고속 미세 진동 방지
- 적용 차량: GRANDEUR_IG, GRANDEUR_HEV_IG, GRANDEUR_FL_IG, GRANDEUR_HEV_FL_IG, K7_YG, K7_HEV_YG

---

## 2. ATOM 횡방향 제어 개선 (latcontrol_atom.py)

### 전환 딜레이 제거
- **변경 전**: 1.5~3초 타이머 기반 샘플링으로 현재 속도/각도 읽기
- **변경 후**: 매 프레임(10ms)마다 즉시 읽기
- `_select_lowdt_speed()`, `_select_lowdt_angle()` 헬퍼 메서드로 리팩토링

### 부드러운 컨트롤러 전환
- **변경 전**: hard switch (즉시 전환, 토크 불연속 발생)
- **변경 후**: 0.5초 FirstOrderFilter 블렌딩 (crossfade)
- `blend_torque_filter`, `blend_angle_filter` 두 개의 필터 적용

### Rate Limiter 추가
- 프레임당 토크 변화량 최대 0.05로 제한 (MAX_TORQUE_RATE)
- 전체 범위(-1~1) 전환에 약 0.4초 소요
- 블렌딩 필터와 이중 안전장치로 작동

---

## 3. 종방향 제어 최적화 (long_mpc.py, longcontrol.py, tunes.py)

### MPC 파라미터
- A_CHANGE_COST: 100 → 40 (선행차 반응 2.5배 개선)
- DANGER_ZONE_COST: 100 → 50 (자연스러운 감속 프로파일)

### decel_damping 활성화
- 47km/h 이상에서 선행차 급감속 시 감속 완충 적용
- damping 계수: [1., 0.15] (최소 15% 감속력 보장)
- damping 타이머: 2.0 × v_ego (빠른 복귀)

### HEV 종방향 PID 튜닝 (OPKR_HEV, 신규)
- **변경 전**: kf=1.0만 사용 (ki=0, kd=0)
- **변경 후**: 속도별 PID 게인 활성화
  - kp: 0.8 → 0.35 (속도별 감소)
  - ki: 0.18 → 0.08 (정상상태 오차 제거)
  - kd: 0.5 → 0.2 (급격한 가감속 억제)
- HEV 회생/유압 제동 전환 구간에서 부드러운 제어

### starting 상태 머신 (master 백포트)
- 정지(stopping) → 출발 시 `starting` 상태를 거쳐 급출발 방지
- `hasattr(CP, 'startingState')` 안전 가드로 하위 호환 유지

---

## 4. Master 최신 기능 백포트 (latcontrol_torque.py, pid.py)

### Torque 컨트롤러 (latcontrol_torque.py)
- **Speed-dependent KP**: 9단계 속도별 테이블 (1km/h: KP=250 → 30km/h: KP=0.8)
- **Jerk-based feedforward**: desired_lateral_jerk 계산 + friction compensation 반영
- **Delay-aware setpoint**: 1초 버퍼 + 0.1초 액추에이터 지연 보상
- **LP filtered jerk**: 1.2Hz cutoff 노이즈 제거
- OPKR live_tune 기능 완전 호환

### PID 컨트롤러 (pid.py)
- `set_limits()` 메서드 추가 (동적 출력 제한값 변경)

---

## 5. 안전성 강화 (safety_hyundai_community.h, values.py 등)

### Panda Safety RX 검증 보강
- CAN ID 902 (WHL_SPD11 휠속도): checksum + counter(max=15) 검증 추가
- CAN ID 1057 (SCC12): checksum + counter(max=15) 검증 추가
- Legacy 차량 하위 호환 유지

### Panda Safety TX 검증 추가
- SCC11/12/13/14: controls_allowed 없이 전송 불가
- FCA11/12: controls_allowed 없이 전송 불가
- Init 함수에서 전역 카운터/버스 변수 완전 초기화

### 파라미터 범위 클램핑 (42개 파라미터)
- `get_safe_param()` 함수: Params 읽기 시 자동 범위 검증
- `SAFETY_LIMITS` 딕셔너리: min/max/default 정의
- 주요 범위:
  - SteerMax: 128~384
  - SteerDeltaUp: 1~5, SteerDeltaDown: 3~10
  - CruiseGap: 최소 0.8초 차간거리 보장
  - TorqueKp: 5~30, TorqueKi: 0~10

### 곡률 제한 안전 범위
- DesiredCurvatureLimit: 0.05~0.20 클램핑

---

## 6. DM(운전자 모니터링) 비활성화

- dmonitoringmodeld 프로세스: enabled=False
- dmonitoringd 프로세스: enabled=False
- controlsd에서 DM 이벤트 제거
- force_decel에서 awarenessStatus 조건 제거

---

## 수정 파일 목록

| 파일 | 변경 내용 |
|------|-----------|
| `selfdrive/car/hyundai/interface.py` | HEV 파라미터, steerRatio, lateral/long 튜닝 선택 |
| `selfdrive/car/hyundai/tunes.py` | OPKR_HEV long 튜닝, TORQUE_HEV_SEDAN lateral 프리셋 |
| `selfdrive/car/hyundai/values.py` | clamp_param, get_safe_param, SAFETY_LIMITS |
| `selfdrive/car/hyundai/carcontroller.py` | get_safe_param 적용 |
| `selfdrive/controls/lib/latcontrol_atom.py` | 전환 블렌딩 + rate limiter |
| `selfdrive/controls/lib/latcontrol_torque.py` | speed KP, jerk FF, delay-aware |
| `selfdrive/controls/lib/longcontrol.py` | decel_damping, starting 상태 |
| `selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py` | A_CHANGE_COST, DANGER_ZONE_COST |
| `selfdrive/controls/lib/pid.py` | set_limits() |
| `selfdrive/controls/lib/drive_helpers.py` | 곡률 제한 클램핑 |
| `selfdrive/monitoring/driver_monitor.py` | DM 관련 변경 |
| `selfdrive/manager/process_config.py` | DM 프로세스 비활성화 |
| `selfdrive/controls/controlsd.py` | DM 이벤트/force_decel 제거 |
| `panda/board/safety/safety_hyundai_community.h` | RX/TX 검증 보강 |
