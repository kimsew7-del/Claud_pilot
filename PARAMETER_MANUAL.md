# Claud_pilot 파라미터 매뉴얼

## 사용법
openpilot 설정 화면에서 각 파라미터를 조정할 수 있습니다.
모든 파라미터는 안전 범위 내로 자동 클램핑됩니다.

---

## 1. 조향(Steering) 파라미터

### SteerMaxBaseAdj / SteerMaxAdj
- **설명**: 최대 조향 토크 값
- **범위**: 128 ~ 384
- **기본값**: 384
- **참고**: panda HYUNDAI_MAX_STEER=384. 384 초과 시 panda에서 자동 차단됨
- **높이면**: 조향력 증가 (급커브 대응력 향상)
- **낮추면**: 조향력 감소 (부드러운 주행, 안정적)

### SteerDeltaUpAdj / SteerDeltaUpBaseAdj
- **설명**: 조향 토크 증가 속도 (프레임당)
- **범위**: 1 ~ 5
- **기본값**: 3
- **높이면**: 조향 반응 빨라짐 (급격한 조향 가능)
- **낮추면**: 조향 반응 느려짐 (부드러운 전환)

### SteerDeltaDownAdj / SteerDeltaDownBaseAdj
- **설명**: 조향 토크 감소 속도 (프레임당)
- **범위**: 3 ~ 10
- **기본값**: 7
- **높이면**: 조향 해제 빨라짐
- **낮추면**: 조향 해제 느려짐

### SteerRatioAdj
- **설명**: 스티어링 기어비 (×0.01)
- **범위**: 1000 ~ 2500 (= 10.0 ~ 25.0)
- **기본값**: 그랜저 IG HEV=1450 (14.5), K7 HEV=1680 (16.8)
- **높이면**: 적은 핸들 입력으로 많은 조향 → 민감해짐
- **낮추면**: 많은 핸들 입력으로 적은 조향 → 안정적

### SteerActuatorDelayAdj
- **설명**: 조향 액추에이터 지연 시간 (×0.01초)
- **범위**: 10 ~ 80 (= 0.1 ~ 0.8초)
- **기본값**: 10 (= 0.1초)
- **참고**: 잘못된 값은 조향 진동/불안정 유발. 차량에 맞는 값 사용 권장

### SteerLimitTimerAdj
- **설명**: 조향 포화 상태 유지 시간 (×0.01초)
- **범위**: 40 ~ 300 (= 0.4 ~ 3.0초)
- **기본값**: 100 (= 1.0초)

### OpkrMaxAngleLimit
- **설명**: LKAS 비활성화 각도 한계
- **범위**: 70 ~ 150도
- **기본값**: 90도
- **참고**: 이 각도를 초과하면 LKAS가 자동 비활성화됨

### OpkrMaxSteeringAngle
- **설명**: smoothSteer 최대 각도
- **범위**: 70 ~ 150도
- **기본값**: 90도

### AvoidLKASFaultMaxAngle
- **설명**: LKAS fault 회피 최대 각도
- **범위**: 50 ~ 150도
- **기본값**: 85도

### AvoidLKASFaultMaxFrame
- **설명**: LKAS fault 회피 최대 프레임 수
- **범위**: 30 ~ 150
- **기본값**: 90

---

## 2. Lateral 튜닝 파라미터

### LateralControlMethod
- **설명**: 횡방향 제어 방식 선택
- **값**: 0=PID, 1=INDI, 2=LQR, 3=TORQUE
- **기본값**: 3 (TORQUE)
- **참고**: 그랜저/K7 HEV는 TORQUE_HEV_SEDAN 프리셋이 자동 적용됨

### TorqueKp
- **설명**: TORQUE P 게인 (×0.1)
- **범위**: 5 ~ 30 (= 0.5 ~ 3.0)
- **기본값**: 10 (= 1.0)
- **높이면**: 조향 반응 빨라짐 (과도하면 진동)
- **낮추면**: 조향 반응 느려짐 (부족하면 차선 이탈)

### TorqueKi
- **설명**: TORQUE I 게인 (×0.1)
- **범위**: 0 ~ 10 (= 0.0 ~ 1.0)
- **기본값**: 1 (= 0.1)
- **높이면**: 누적 오차 빠르게 보정 (과도하면 폭주)
- **낮추면**: 누적 오차 천천히 보정

### TorqueKf
- **설명**: TORQUE 피드포워드 게인 (×0.1)
- **범위**: 5 ~ 25 (= 0.5 ~ 2.5)
- **기본값**: 10 (= 1.0)

### TorqueFriction
- **설명**: TORQUE 마찰 보상 (×0.001)
- **범위**: 0 ~ 200 (= 0.0 ~ 0.2)
- **기본값**: 65 (= 0.065)
- **참고**: TORQUE_HEV_SEDAN에서는 ×1.3 계수 자동 적용

### TorqueMaxLatAccel
- **설명**: 최대 횡가속도 (×0.1 m/s²)
- **범위**: 15 ~ 40 (= 1.5 ~ 4.0)
- **기본값**: 27 (= 2.7 m/s²)
- **참고**: EU 기준 3.0m/s², TORQUE_HEV_SEDAN에서는 ×0.85 자동 적용

### DesiredCurvatureLimit
- **설명**: 최대 요청 곡률 (×0.01)
- **범위**: 5 ~ 20 (= 0.05 ~ 0.20)
- **기본값**: 10 (= 0.10)
- **높이면**: 더 급한 커브에서도 조향 유지
- **낮추면**: 완만한 커브에서만 조향

---

## 3. ATOM 다중 횡방향 제어

### MultipleLateralUse
- **설명**: ATOM 다중 컨트롤러 모드
- **값**: 0=SPEED_LOWDT, 1=ANGLE_LOWDT, 2=ANGLE_INTERP, 3=SPEED_INTERP
- **기본값**: 2
- **0 (SPEED_LOWDT)**: 속도 구간별 컨트롤러 전환 (0.5초 블렌딩 적용)
- **1 (ANGLE_LOWDT)**: 조향각 구간별 컨트롤러 전환 (0.5초 블렌딩 적용)
- **2 (ANGLE_INTERP)**: 조향각 기반 3개 컨트롤러 출력 보간 (권장)
- **3 (SPEED_INTERP)**: 속도 기반 3개 컨트롤러 출력 보간

### MultipleLateralOpS
- **설명**: 속도 구간별 사용할 컨트롤러 번호 (SPEED 모드)
- **형식**: "컨트롤러1,컨트롤러2,컨트롤러3"
- **값**: 0=PID, 1=INDI, 2=LQR, 3=TORQUE

### MultipleLateralSpd
- **설명**: 속도 전환 기준점 (SPEED 모드, km/h)
- **형식**: "속도1,속도2"

### MultipleLateralOpA
- **설명**: 조향각 구간별 사용할 컨트롤러 번호 (ANGLE 모드)

### MultipleLateralAng
- **설명**: 조향각 전환 기준점 (ANGLE 모드, 도)

---

## 4. 종방향(Longitudinal) 파라미터

### CruiseGap1 ~ CruiseGap4
- **설명**: 차간거리 4단계 (×0.1초)
- **범위**: Gap1: 8~20, Gap2: 10~25, Gap3: 12~30, Gap4: 14~40
- **기본값**: Gap1=12(1.2초), Gap2=13(1.3초), Gap3=14(1.4초), Gap4=16(1.6초)
- **참고**: 숫자가 클수록 앞차와의 거리가 멀어짐
- **팁**: Gap1은 최소 0.8초(8) 이상 권장 (추돌 방지)

### DynamicTRGap
- **설명**: 속도 기반 동적 차간거리 조절 활성화
- **값**: 0=비활성, 1=활성
- **참고**: 활성 시 DynamicTRSpd/DynamicTRSet에 따라 자동 조절

### StoppingDist
- **설명**: 정지 거리 (×0.1m)
- **범위**: 20 ~ 60 (= 2.0 ~ 6.0m)
- **기본값**: 38 (= 3.8m)
- **높이면**: 앞차와 더 멀리서 정지
- **낮추면**: 앞차에 더 가깝게 정지

### AutoEnable
- **설명**: 자동 활성화 (기어 D + 속도 조건 시 자동 크루즈 시작)
- **값**: 0=비활성, 1=활성
- **기본값**: 1

### AutoEnableSpeed
- **설명**: 자동 활성화 최소 속도 (km/h)
- **범위**: 5 ~ 30
- **기본값**: 9
- **참고**: 이 속도 이상에서 자동 크루즈 시작

### OpkrVariableCruise
- **설명**: 가변 크루즈 속도 설정
- **값**: 0=비활성, 1=활성
- **참고**: 내비/속도 카메라 연동 시 자동 속도 조절

---

## 5. 속도카메라/내비 연동 파라미터

### OPKRNaviSelect
- **설명**: 내비게이션 소스 선택
- **값**: 1=기본, 3=Waze, 4=eNavi, 5=eNavi+Waze
- **참고**: 외부 내비 앱에서 속도 카메라/제한 속도 정보 수신

### OSMSpeedLimitEnable
- **설명**: OpenStreetMap 속도 제한 연동
- **값**: 0=비활성, 1=활성

### CruiseSetwithRoadLimitSpeedEnabled
- **설명**: 도로 제한속도에 맞춰 크루즈 자동 설정
- **값**: 0=비활성, 1=활성

### SafetyCamDecelDistGain
- **설명**: 안전카메라 감속 거리 게인 (×0.01)
- **기본값**: 0
- **높이면**: 안전카메라 앞에서 더 일찍 감속 시작

---

## 6. 안전 관련 파라미터

### RadarDisable
- **설명**: SCC ECU 비활성화 (openpilot이 종방향 직접 제어)
- **값**: 0=비활성(순정 SCC 사용), 1=활성(openpilot 제어)
- **주의**: 활성 시 community safety 모드로 전환됨

### RTShield
- **설명**: 실시간 보호 프로세스
- **값**: 0=비활성, 1=활성
- **기본값**: 1

### ComIssueGone
- **설명**: 통신 문제 무시 플래그
- **값**: 0=정상, 1=무시
- **주의**: 1로 설정 시 통신 오류 경고가 무시됨

---

## 7. UI/UX 파라미터

### OpkrAutoScreenOff
- **설명**: 자동 화면 끄기 (초)
- **값**: -1=비활성, 0~600초
- **기본값**: -1

### OpkrUIBrightness
- **설명**: UI 밝기 (%)
- **값**: 0~100
- **기본값**: 0 (자동)

### DebugUi1 / DebugUi2 / DebugUi3
- **설명**: 디버그 정보 표시
- **값**: 0=비활성, 1=활성
- **참고**: 차량 상태, SCC 데이터, 제어 값 등 표시

---

## 8. 배터리 관리 파라미터

### OpkrBatteryChargingControl
- **설명**: 배터리 충전 제어 활성화
- **값**: 0=비활성, 1=활성

### OpkrBatteryChargingMin
- **설명**: 충전 시작 배터리 잔량 (%)
- **기본값**: 50

### OpkrBatteryChargingMax
- **설명**: 충전 중지 배터리 잔량 (%)
- **기본값**: 60
- **참고**: 배터리 수명 보호를 위해 50~60% 범위 권장

---

## 9. 안전 범위 일람표

아래는 `get_safe_param()`에 의해 자동 클램핑되는 파라미터 범위입니다.
사용자가 범위 밖 값을 입력해도 자동으로 가장 가까운 유효값으로 보정됩니다.

| 파라미터 | 최소 | 최대 | 기본값 | 단위/비고 |
|----------|------|------|--------|-----------|
| SteerMaxBaseAdj | 128 | 384 | 384 | 토크 |
| SteerMaxAdj | 128 | 384 | 384 | 토크 |
| SteerDeltaUpAdj | 1 | 5 | 3 | 프레임당 |
| SteerDeltaUpBaseAdj | 1 | 5 | 3 | 프레임당 |
| SteerDeltaDownAdj | 3 | 10 | 7 | 프레임당 |
| SteerDeltaDownBaseAdj | 3 | 10 | 7 | 프레임당 |
| OpkrMaxAngleLimit | 70 | 150 | 90 | 도 |
| OpkrMaxSteeringAngle | 70 | 150 | 90 | 도 |
| SteerRatioAdj | 1000 | 2500 | 1550 | ×0.01 |
| SteerActuatorDelayAdj | 10 | 80 | 10 | ×0.01초 |
| SteerLimitTimerAdj | 40 | 300 | 100 | ×0.01초 |
| CruiseGap1 | 8 | 20 | 12 | ×0.1초 |
| CruiseGap2 | 10 | 25 | 13 | ×0.1초 |
| CruiseGap3 | 12 | 30 | 14 | ×0.1초 |
| CruiseGap4 | 14 | 40 | 16 | ×0.1초 |
| StoppingDist | 20 | 60 | 38 | ×0.1m |
| AutoEnableSpeed | 5 | 30 | 9 | km/h |
| TorqueKp | 5 | 30 | 10 | ×0.1 |
| TorqueKi | 0 | 10 | 1 | ×0.1 |
| TorqueKf | 5 | 25 | 10 | ×0.1 |
| TorqueFriction | 0 | 200 | 65 | ×0.001 |
| TorqueMaxLatAccel | 15 | 40 | 27 | ×0.1 m/s² |
| DesiredCurvatureLimit | 5 | 20 | 10 | ×0.01 |
| AvoidLKASFaultMaxAngle | 50 | 150 | 85 | 도 |
| AvoidLKASFaultMaxFrame | 30 | 150 | 90 | 프레임 |

---

## 10. 추천 설정 (그랜저 IG HEV / K7 HEV)

### 기본 추천 (변경 없이 사용)
모든 값이 이미 최적화되어 있으므로 기본값 그대로 사용 권장

### 고속도로 위주
- CruiseGap: Gap1=12, Gap2=14, Gap3=16, Gap4=20
- TorqueMaxLatAccel: 25 (약간 보수적)
- OpkrVariableCruise: 1 (가변 크루즈 활성)
- OSMSpeedLimitEnable: 1

### 시내 주행 위주
- CruiseGap: Gap1=10, Gap2=12, Gap3=14, Gap4=16
- StoppingDist: 30 (짧은 정지 거리)
- AutoEnableSpeed: 15 (저속 자동 활성화 방지)

### 커브 많은 도로
- SteerMaxAdj: 384 (최대)
- TorqueKp: 12 (약간 높게)
- DesiredCurvatureLimit: 15 (커브 대응력 향상)
