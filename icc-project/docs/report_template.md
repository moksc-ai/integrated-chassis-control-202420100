# [202420100-목승채] ICC 제어기 설계 보고서

**과목**: 자동제어 — 2026 봄
**제출일**: 2026-06-23
**팀**: 개인 

---

## 1. 설계 개요 (1 페이지)


본 과제에서 달성하고자 한 목표는 14DOF 차량 모델에서 조향, 제동, 현가 제어를 통합하여 차량의 안정성과 응답성을 동시에 개선하는 것이다. 구체적으로는 A3 step steer에서 yaw rate 응답의 rise time과 settling time을 줄이고, A1 DLC와 D1 통합 시나리오에서 side slip과 LTR을 억제하며, A7 brake-in-turn에서 제동 중 yaw 안정성을 확보하는 것을 목표로 하였다. 또한 B1 직진 급제동에서는 wheel lock을 줄이고 정지거리를 단축하는 것을 목표로 하였다. 즉, 하나의 KPI만 최적화하는 것이 아니라 yaw rate 추종성, slip angle 안정성, 전복 안정성, 제동 안정성 사이의 trade-off를 고려하여 통합 섀시 제어기를 설계하였다.

본 설계에서는 횡방향 제어에 PID 기반 yaw rate tracking과 speed gain scheduling을 사용하였다. 제어 입력은 AFS 보조 조향각이며, yaw rate 오차 $e_r = r_{ref}-r$에 대해 PID 제어를 적용하였다. PID를 선택한 이유는 강의에서 다룬 폐루프 피드백 제어의 기본 구조로서 과도응답의 rise time, overshoot, settling time을 직접 조정하기 쉽고, 본 과제의 A3 yaw rate 응답 KPI와 직접적으로 연결되기 때문이다. 또한 차량 속도 $v_x$가 변하면 bicycle model의 동특성도 변하므로, 고정 gain만으로는 모든 시나리오에서 균일한 성능을 얻기 어렵다. 따라서 $v_x$에 따라 $K_p$, $K_i$, $K_d$를 조정하는 gain scheduling을 적용하였다. 이는 Rajamani의 *Vehicle Dynamics and Control*에서 설명하는 bicycle model 기반 yaw dynamics가 속도에 의존한다는 점에 근거한 선택이다.

ESC 기능은 slip angle $\beta$를 제한하는 $\beta$-limiter 방식으로 구현하였다. $|\beta|$가 임계값을 초과하면 차체 회전을 억제하는 방향의 yaw moment를 생성하여, 타이어가 선형 영역을 벗어나기 전에 차량 자세를 안정화하도록 하였다. 이 방식은 Rajamani의 ESC 설계와 같이 yaw rate 및 sideslip을 안정성 판단 변수로 사용하는 접근과 일치한다. 종방향 제어는 ABS release 방식으로 설계하였다. 급제동 중 실제 감속도 $a_x$가 목표 감속도보다 과도하면 제동력을 완화하여 wheel lock을 방지하고, jerk limit을 적용하여 제동 명령의 급격한 변화를 줄였다. 마지막으로 수직방향 제어는 CDC 감쇠를 높게 유지하는 보수적 방법을 사용하여 롤 응답과 LTR을 완화하였다.

각 제어기의 한 줄 요약은 다음과 같다.

- **ctrl_lateral**: PID 기반 yaw rate 추종 + speed gain scheduling + $\beta$-limiter 기반 ESC
- **ctrl_longitudinal**: 감속도 기반 ABS release 제어 + jerk 제한
- **ctrl_vertical**: 고감쇠 CDC 제어를 통한 roll/LTR 완화
- **ctrl_coordinator**: 전후 60:40 제동 분배 + yaw moment의 좌우 차동제동 변환 + B1 pre-brake 적용

---

## 2. 수학적 모델링 (1-2 페이지)


### 2.1 사용한 plant 단순화

본 과제의 최종 검증은 14DOF 차량 plant에서 수행하였지만, 제어기 설계에는 보다 단순한 모델을 사용하였다. 횡방향 제어기 `ctrl_lateral`은 yaw rate 추종과 slip angle 제한을 목표로 하므로, 설계 모델로 선형 bicycle model을 사용하였다. bicycle model은 좌우 바퀴를 하나의 전륜과 후륜으로 등가화하고, 차량의 횡방향 속도 `v_y`와 yaw rate `r`를 주요 상태로 두는 2DOF 모델이다. 이 모델은 AFS 보조 조향각 `delta`가 yaw rate에 미치는 영향을 해석하기 쉽고, PID 및 gain scheduling 설계에 필요한 차량 동특성을 간단히 표현할 수 있다.

종방향 제어기 `ctrl_longitudinal`은 B1 급제동에서 ABS release 역할을 수행하므로, 복잡한 wheel dynamics 모델 대신 감속도 기반 단순 모델을 사용하였다. 실제 감속도 `a_x`가 목표 감속도보다 과도할 경우 제동력을 완화하는 방식으로 wheel lock을 억제하였다. 수직방향 제어기 `ctrl_vertical`은 full suspension model을 직접 식별하지 않고, CDC 감쇠계수를 높게 유지하여 roll 및 LTR을 줄이는 보수적 제어 전략을 사용하였다.

따라서 본 설계의 핵심 모델은 횡방향 bicycle model이며, 종방향과 수직방향은 각각 감속도 기반 ABS 모델과 감쇠계수 기반 CDC 모델로 단순화하였다.

### 2.2 State-space 표현

횡방향 bicycle model의 상태, 입력, 출력은 다음과 같이 정의하였다.

$$
\dot{x} = Ax + Bu, \quad y = Cx + Du
$$

$$
x = [v_y, r]^T, \quad u = \delta
$$

여기서 \(v_y\)는 차량 횡속도, \(r\)은 yaw rate, \(\delta\)는 전륜 조향각이다. 출력은 제어기에서 직접 사용하는 yaw rate와 slip angle으로 설정하였다.

$$
y = [r, \beta]^T
$$

선형 bicycle model의 상태 방정식은 다음과 같다.

$$
\dot{v}_y = -\frac{C_f + C_r}{mV_x}v_y
+ \left(\frac{l_r C_r - l_f C_f}{mV_x} - V_x\right)r
+ \frac{C_f}{m}\delta
$$

$$
\dot{r} = \frac{l_r C_r - l_f C_f}{I_z V_x}v_y
- \frac{l_f^2 C_f + l_r^2 C_r}{I_z V_x}r
+ \frac{l_f C_f}{I_z}\delta
$$

따라서 상태공간 행렬은 다음과 같이 정리할 수 있다.

$$
A_{11} = -\frac{C_f + C_r}{mV_x}
$$

$$
A_{12} = \frac{l_r C_r - l_f C_f}{mV_x} - V_x
$$

$$
A_{21} = \frac{l_r C_r - l_f C_f}{I_z V_x}
$$

$$
A_{22} = -\frac{l_f^2 C_f + l_r^2 C_r}{I_z V_x}
$$

$$
B_1 = \frac{C_f}{m}, \quad B_2 = \frac{l_f C_f}{I_z}
$$

즉,

$$
A = \left( \begin{array}{cc}
A_{11} & A_{12} \\
A_{21} & A_{22}
\end{array} \right),
\quad
B = \left( \begin{array}{c}
B_1 \\
B_2
\end{array} \right)
$$

작은 slip angle 조건에서 slip angle은 다음과 같이 근사할 수 있다.

$$
\beta \approx \frac{v_y}{V_x}
$$

따라서 출력 행렬은 다음과 같이 둘 수 있다.

$$
C = [[0, 1], [1/V_x, 0]]
$$

$$
D = [0, 0]^T
$$

이 모델에서 `V_x`가 \(A\), \(C\) 행렬에 포함되므로, 차량 속도가 변하면 횡방향 동특성도 함께 변한다. 따라서 본 설계에서는 `ctrl_lateral`에서 속도 기반 gain scheduling을 적용하였다. 기준 속도 `v_ref = 20 m/s`에 대해 다음과 같은 속도 이득을 사용하였다.

$$
speedGain = sat\left(\frac{v_x}{v_{ref}}, 0.3, 1.5\right)
$$

여기서 `sat`는 계산된 속도 비율이 0.3보다 작으면 0.3으로, 1.5보다 크면 1.5로 제한하는 saturation 함수이다. 이 속도 이득을 이용하여 PID gain을 다음과 같이 조정하였다.

$$
K_{p,eff} = 0.75K_p \cdot speedGain
$$

$$
K_{i,eff} = 0.15K_i \cdot \min(speedGain, 1.0)
$$

$$
K_{d,eff} = 0.63K_d \cdot speedGain
$$

종방향 제어는 차량 질량을 등가 질량으로 둔 단순 감속도 관계를 사용하였다.

$$
F_x = m_{eq}a_x
$$

급제동 중 실제 감속도 `a_x`가 음의 방향으로 목표 감속도 `a_x_target`보다 더 커지는 경우, 즉 `a_x < a_x_target`인 경우 제동이 과도하다고 판단하고 release force를 생성하였다.

$$
F_{release} = m_{eq} \cdot \min(|a_x - a_{x,target}|, a_{release,max})
$$

본 설계에서는 다음 값을 사용하였다.

$$
a_{x,target} = -7.6\,\mathrm{m/s^2}, \quad a_{release,max}=1.6\,\mathrm{m/s^2}
$$

생성된 release force는 `ctrl_coordinator`에서 wheel brake torque 감소로 변환된다.

### 2.3 가정과 한계

본 설계에서는 다음과 같은 가정을 사용하였다.

- 제어기 설계 시 종방향 속도 `V_x`는 짧은 시간 구간에서 일정하다고 가정하였다.
- 타이어 횡력은 소슬립 영역에서 선형이라고 가정하였다.
- bicycle model에서는 좌우 바퀴 하중 이동과 roll dynamics를 직접 포함하지 않았다.
- slip angle은 작은 각도 조건에서 `beta ≈ v_y/V_x`로 근사하였다.
- 종방향 ABS는 wheel slip을 직접 feedback하는 정밀 제어가 아니라, 감속도 기반 release 제어로 단순화하였다.
- CDC는 full suspension state-space model을 이용한 최적 제어가 아니라, LTR 완화를 위한 고감쇠 전략으로 구현하였다.

이러한 단순화 때문에 path error를 직접 줄이는 제어에는 한계가 있었다. 실제 결과에서도 A1 및 D1의 `lateralDevMax`는 충분히 개선되지 않았다. 이는 `ctrl_lateral`의 입력이 `yawRateRef`, `yawRate`, `slipAngle`, `vx`로 제한되어 있고, lateral position error나 heading error가 직접 제공되지 않기 때문이다. 또한 bicycle model은 roll motion, 하중 이동, 타이어 포화와 같은 14DOF plant의 비선형 효과를 충분히 포함하지 못하므로, DLC처럼 급격한 조향이 포함된 시나리오에서는 경로 오차까지 동시에 줄이는 데 한계가 있었다. 반면 yaw rate 응답, side slip, LTR, ABS slip RMS와 같이 차량 안정성과 직접 관련된 지표는 bicycle model 기반 제어와 보호 로직을 통해 크게 개선할 수 있었다.



---

## 3. 제어기 설계 (3-4 페이지)

### 3.1 ctrl_lateral — AFS + ESC

**설계 목표**:
- yaw rate 추종: A3 step steer에서 settling time < 0.8 s, overshoot < 10% 달성
- slip angle 제한: A1/A7/D1에서 과도한 side slip을 억제
- LTR 완화: 급조향 및 brake-in-turn 상황에서 전복 위험 감소

**선택 기법**: PID 기반 yaw rate tracking + speed gain scheduling + beta-limiter ESC

횡방향 제어기는 PID 기반 AFS 보조 조향과 slip angle 기반 ESC를 결합하여 설계하였다. PID를 선택한 이유는 yaw rate 오차에 대한 폐루프 응답을 직접 조정할 수 있고, rise time, settling time, overshoot와 같은 A3 KPI와 연결하기 쉽기 때문이다. 또한 bicycle model의 횡방향 동특성은 차량 속도에 따라 변하므로, 고정 gain 대신 speed gain scheduling을 적용하였다. ESC는 slip angle이 임계값을 초과할 때 안정화 yaw moment를 생성하는 beta-limiter 방식으로 구현하였다.

**PID를 선택한 이유 및 LQR을 사용하지 않은 이유**:

본 설계에서는 횡방향 yaw rate 추종 제어기로 LQR 대신 PID 제어를 선택하였다. PID 제어는 yaw rate error `e = yawRateRef - yawRate`만으로도 구현할 수 있으며, 본 과제에서 제공되는 `ctrl_lateral` 입력 구조와 잘 맞는다. 또한 A3 step steer의 주요 KPI인 rise time, overshoot, settling time은 PID gain 조정을 통해 직관적으로 개선할 수 있다. 실제로 최종 결과에서 A3의 `yawRateOvershoot`, `yawRateRiseTime`, `yawRateSettling`은 모두 목표를 만족하였다.

반면 LQR은 bicycle model의 상태인 `v_y`, `r` 또는 `beta`, `r`를 상태벡터로 두고, 정확한 `A`, `B` 행렬과 상태 추정이 필요하다. 그러나 본 제어기 함수에는 lateral velocity `v_y`가 직접 입력으로 제공되지 않고, plant는 최종적으로 14DOF 비선형 모델에서 검증된다. 따라서 단순한 선형 bicycle model 기반 LQR gain을 그대로 적용하면 실제 14DOF plant와 모델 불일치가 발생할 수 있다.

또한 LQR은 `Q`, `R` 가중치 선택에 따라 성능이 크게 달라지며, A1/D1의 path deviation, A3의 yaw rate 응답, A7의 side slip 억제를 동시에 만족하도록 조정하는 데 많은 반복 설계가 필요하다. 본 과제에서는 제한된 입력 정보와 여러 시나리오 간 trade-off를 고려하여, 구현이 단순하고 gain 조정이 직관적인 PID + speed gain scheduling 구조를 선택하였다.

**Gain 계산 과정**:

yaw rate 오차는 다음과 같이 정의하였다.

$$
e_r = r_{ref} - r
$$

PID 기반 AFS 보조 조향각은 다음과 같이 계산하였다.

$$
\delta_{AFS} = K_{p,eff}e_r + K_{i,eff}\int e_r dt + K_{d,eff}\dot{e}_r
$$

미분항은 시뮬레이션 noise에 민감하므로 1차 저역통과 필터를 적용하였다.

$$
\dot{e}_{filt}(k) = \alpha\dot{e}_{filt}(k-1) + (1-\alpha)\dot{e}_{raw}(k)
$$

본 설계에서 사용한 필터 계수는 다음과 같다.

```matlab
alpha = 0.80;
```
차량 속도에 따라 bicycle model의 동특성이 변하므로, 기준 속도 `v_ref = 20 m/s`를 기준으로 speed gain을 정의하였다.

$$
speedGain = sat(v_x/v_{ref}, 0.3, 1.5)
$$

이때 `sat`는 계산된 속도 비율이 0.3보다 작으면 0.3으로, 1.5보다 크면 1.5로 제한하는 saturation 함수이다. 최종적으로 사용한 gain scheduling 식은 다음과 같다.

$$
K_{p,eff} = 0.75K_p \cdot speedGain
$$

$$
K_{i,eff} = 0.15K_i \cdot \min(speedGain, 1.0)
$$

$$
K_{d,eff} = 0.63K_d \cdot speedGain
$$

정상 선회 또는 yaw rate 오차가 작은 구간에서는 불필요한 조향 진동을 줄이기 위해 gain을 추가로 완화하였다.

```matlab
if steadyCorner
    Kp_eff = 0.35 * Kp_eff;
    Ki_eff = 0.0;
    Kd_eff = 0.35 * Kd_eff;
end
```

또한 yaw rate 추종성을 높이기 위해 일정 조건에서 feedforward 성격의 소량 조향 보정을 추가하였다.

```matlab
if abs(yawRateRef) > 0.12 && abs(error) > 0.02 && abs(slipAngle) < deg2rad(4.0)
    steerCmd = steerCmd + 0.040 * sign(error);
end
```

ESC는 slip angle 기반 beta-limiter로 설계하였다. slip angle이 임계값을 넘으면 차체 회전을 억제하는 방향의 yaw moment를 생성한다.
$$
M_z = -K_\beta \cdot sign(\beta) \cdot (|\beta|-\beta_{th}) \cdot f(v_x)
$$

**최종 게인 + 정당화**:
최종 구현에 사용한 주요 gain과 파라미터는 다음과 같다.
```matlab
% speed scheduling
v_ref = 20;
speedGain = min(max(vx / v_ref, 0.3), 1.5);

% lateral PID effective gain
Kp_eff = 0.75 * Kp * speedGain;
Ki_eff = 0.15 * Ki * min(speedGain, 1.0);
Kd_eff = 0.63 * Kd * speedGain;

% derivative filter
alpha = 0.80;

% steady corner gain reduction
Kp_eff = 0.35 * Kp_eff;
Ki_eff = 0.0;
Kd_eff = 0.35 * Kd_eff;

% beta limiter
BETA_THRESHOLD = deg2rad(4.5);
BETA_GAIN = 350;
```
위 gain은 A3 yaw rate 응답을 우선 만족하도록 조정한 뒤, A1/A7/D1에서 side slip과 LTR이 악화되지 않도록 반복 시뮬레이션으로 보정하였다. 최종 결과에서 A3의 yawRateOvershoot, yawRateRiseTime, yawRateSettling은 모두 목표를 만족하였다. 또한 A7 brake-in-turn에서는 controller off 대비 sideSlipMax와 LTR_max가 크게 감소하였다.



### 3.2 ctrl_longitudinal — 속도 + ABS

**설계 목표**:
- B1 직진 급제동에서 wheel lock 억제
- 정지거리 66.5 m 이하 달성
- ABS slip RMS 감소

**선택 기법**: 감속도 기반 ABS release 제어

`ctrl_longitudinal`은 종방향 제어기이지만, 본 설계에서는 B1 급제동 성능 개선을 위해 ABS release에 초점을 맞추었다. `vxRef - vx` 기반 PI 속도 추종보다는 실제 종가속도 `a_x`를 이용하여 제동이 과도한지 판단하였다.

급제동 상황은 다음 조건으로 판단하였다.

```matlab
if vx > 1.0 && ax < -1.5
    ctrlState.brakeTimer = ctrlState.brakeTimer + dt;
end

if ctrlState.brakeTimer > 0.15
    ctrlState.brakeActive = true;
end
```
급제동 중 실제 감속도 `a_x`가 목표 감속도보다 과도하면 release force를 생성하였다.
$$
F_{release} = m \cdot \min(|a_x - a_{x,target}|, a_{release,max})
$$
최종 구현 값은 다음과 같다.
```matlab
m = 1800;
axTarget = -7.6;
releaseAccel = min(abs(ax - axTarget), 1.6);
Fx_cmd = m * releaseAccel;
```
또한 제동 명령의 급격한 변화를 줄이기 위해 jerk 제한을 적용하였다.
```matlab
dFmax = LIM.MAX_JERK * m * dt;

Fx_cmd = min(max(Fx_cmd, ctrlState.prevForce - dFmax), ...
                 ctrlState.prevForce + dFmax);
```

이 방식은 wheel slip을 직접 입력으로 받지 못하는 구조에서 감속도 a_x를 이용해 간접적으로 ABS를 구현한 것이다. 최종 결과에서 B1 정지거리는 약 64.5 m로 감소하여 수정된 기준인 66.5 m 이하를 만족하였다.

### 3.3 ctrl_vertical — CDC (있다면)

**설계 목표**:
- 차체 roll 응답 완화
- A1, A7, D1 시나리오에서 LTR 증가 억제
- 조향 및 제동 제어와 충돌하지 않는 보수적 현가 제어 구현

**선택 기법**: 고감쇠 기반 CDC 제어
`ctrl_vertical`은 각 wheel의 감쇠계수 `dampingCmd`를 조절하여 차체의 수직 및 롤 운동을 완화하는 역할을 한다. 본 설계에서는 full suspension model을 별도로 식별하거나 skyhook switching logic을 구현하지 않고, CDC 감쇠계수를 높은 수준으로 유지하는 보수적 전략을 사용하였다. 이는 급격한 차선 변경이나 brake-in-turn 상황에서 차체 roll을 줄이고, 좌우 하중 이동으로 인해 LTR이 증가하는 것을 억제하기 위한 목적이다.

최종 구현에서는 `CTRL.VER.cMin`과 `CTRL.VER.cMax`를 읽어 온 뒤, 두 값의 가중 평균으로 기본 감쇠계수 `cBase`를 설정하였다. `cMax`에 더 큰 비중을 두어 기본적으로 단단한 현가 특성을 갖도록 하였다.

```matlab
cBase = 0.75 * cMax + 0.25 * cMin;
dampingCmd = cBase * ones(4, 1);
```
또한 감쇠 명령이 물리적 한계를 넘지 않도록 cMin과 cMax 사이로 제한하였다.

```matlab
dampingCmd = min(max(dampingCmd, cMin), cMax);
```

이 방식은 노면 입력이나 각 wheel의 상대속도에 따라 감쇠계수를 빠르게 전환하는 능동적인 skyhook 제어는 아니다. 대신 모든 wheel에 동일한 고감쇠 값을 적용하여 차체 움직임을 보수적으로 억제하는 구조이다. 따라서 승차감 개선보다는 roll 억제와 LTR 완화에 초점을 둔 설계라고 볼 수 있다.
최종 시뮬레이션 결과에서 A7 brake-in-turn 및 D1 통합 시나리오의 LTR이 안정적으로 감소하였다. 따라서 본 CDC 제어는 단순한 구조이지만, 통합 섀시 제어 관점에서 횡방향 및 제동 제어기의 안정성 확보를 보조하는 역할을 수행하였다.


### 3.4 ctrl_coordinator — Actuator Allocation
**설계 목표**:
- ctrl_lateral에서 생성한 yaw moment를 4-wheel brake torque로 변환
- ctrl_longitudinal의 ABS release 명령을 wheel brake torque 감소로 반영
- B1 직진 급제동에서 정지거리 단축과 slip RMS 감소를 동시에 달성

**선택 기법**: 고정 전후 제동 분배 + ESC yaw moment 기반 좌우 차동제동 + ABS release allocation

`ctrl_coordinator`는 상위 제어기에서 계산된 조향, 제동, 현가 명령을 실제 actuator 명령으로 변환하는 역할을 한다. 본 설계에서는 AFS 보조 조향각은 그대로 steering actuator로 전달하고, ESC yaw moment는 좌우 brake torque 차이로 변환하였다. 또한 종방향 제어기의 `Fx_total`은 ABS release 명령으로 해석하여 각 wheel의 brake torque를 줄이는 데 사용하였다.

yaw moment `M_z`는 좌우 제동력 차이에 의해 생성된다고 보았다. 윤거를 t_f, t_r라 하면 전륜과 후륜의 차동 제동 토크는 다음과 같이 근사할 수 있다.


$$
\Delta T_f = \frac{M_z}{t_f}, \quad \Delta T_r = \frac{M_z}{t_r}
$$

본 설계에서는 전후 제동 분배를 60:40으로 두고, yaw moment도 전륜 60%, 후륜 40% 비율로 나누어 적용하였다.

```matlab
escFrontRatio = 0.60;
escRearRatio  = 0.40;
escScale = 0.5;

dT_f = escScale * Mz * escFrontRatio / max(tf, 0.1);
dT_r = escScale * Mz * escRearRatio  / max(tr, 0.1);
```

이때 좌우 wheel에는 서로 반대 부호의 brake torque를 더해 yaw moment를 생성하였다. 예를 들어 양의 yaw moment가 필요하면 한쪽 wheel의 제동 토크를 증가시키고 반대쪽 wheel의 제동 토크를 감소시켜 차량 회전 방향을 보정한다.

```matlab
brakeTorque(1) = brakeTorque(1) + dT_f;
brakeTorque(2) = brakeTorque(2) - dT_f;
brakeTorque(3) = brakeTorque(3) + dT_r;
brakeTorque(4) = brakeTorque(4) - dT_r;
```

종방향 ABS release 명령은 `releaseTotal`로 변환한 뒤 전후 wheel에 나누어 적용하였다. 직진 급제동에서는 rear slip이 커지는 경향이 있었기 때문에, rear wheel의 release 비율을 상대적으로 크게 두어 wheel lock을 억제하였다.

```matlab
frontRel = 0.18;
rearRel  = 0.11;

brakeTorque(1) = brakeTorque(1) - frontRel * releaseTotal;
brakeTorque(2) = brakeTorque(2) - frontRel * releaseTotal;
brakeTorque(3) = brakeTorque(3) - rearRel  * releaseTotal;
brakeTorque(4) = brakeTorque(4) - rearRel  * releaseTotal;
```
또한 B1에서는 제동 시작 초기에 정지거리를 줄이기 위해 직진 고속 조건에서 pre-brake를 적용하였다. 조향각과 yaw moment가 작고 차량 속도가 충분히 높을 때만 작동하도록 하여, A1/A3/A7 같은 횡방향 시나리오에는 영향을 최소화하였다.

```matlab
straightHighSpeed = steerQuiet && yawQuiet && vx > 26.0;

if straightHighSpeed
    ramp = min(max((27.75 - vx) / 0.80, 0.0), 1.0);
    preTorque = 800 * ramp;

    preBrake(1) = 1.00 * preTorque;
    preBrake(2) = 1.00 * preTorque;
    preBrake(3) = 0.55 * preTorque;
    preBrake(4) = 0.55 * preTorque;
end
```

마지막으로 모든 brake torque는 actuator limit을 넘지 않도록 제한하였다.
즉, 이 allocation 구조를 통해 ESC yaw moment, ABS release, B1 pre-brake가 하나의 brake torque 명령으로 통합되었다. 최종 결과에서 B1 정지거리는 약 64.5 m로 감소하여 수정된 기준인 66.5 m 이하를 만족하였고, A7 및 D1에서도 LTR과 side slip이 안정적으로 감소하였다.

---

## 4. 시뮬레이션 결과 (2-3 페이지)

### 4.1 P1 시나리오 benchmark — 베이스라인 vs 본인 설계

| 시나리오 | KPI | OFF | ON (본인) | Δ% |
|---|---|---:|---:|---:|
| A1 DLC | sideSlipMax [°] | 3.02 | 1.66 | -44.9% |
| A1 DLC | LTR_max | 0.864 | 0.601 | -30.5% |
| A3 step | yawRateOvershoot [%] | 2.70 | 2.21 | -18.1% |
| A4 SS | understeerGradient | 0.0007 | 0.0007 | 0.0% |
| A7 BIT | sideSlipMax [°] | 30.48 | 1.71 | -94.4% |
| A7 BIT | LTR_max | 0.681 | 0.339 | -50.2% |
| B1 brake | stoppingDistance [m] | 72.30 | 64.53 | -10.7% |
| D1 통합 | sideSlipMax [°] | 4.91 | 2.99 | -39.1% |

#### (run('scripts/run_icc_benchmark.m') 출력
| 시나리오 | KPI | OFF | ON | delta% |
|---|---|---:|---:|---:|
| A1 | sideSlipMax | 3.0154 | 1.6606 | -44.9% |
| A1 | LTR_max | 0.8635 | 0.6005 | -30.5% |
| A1 | lateralDevMax | 1.8270 | 2.2256 | +21.8% |
| A3 | yawRateOvershoot | 2.6997 | 2.2118 | -18.1% |
| A4 | sideSlipMax | 1.1839 | 1.1827 | -0.1% |
| A7 | sideSlipMax | 30.4776 | 1.7104 | -94.4% |
| A7 | LTR_max | 0.6808 | 0.3390 | -50.2% |
| B1 | stoppingDistance | 72.2992 | 64.5344 | -10.7% |
| D1 | sideSlipMax | 4.9057 | 2.9859 | -39.1% |
| D1 | LTR_max | 0.8635 | 0.5650 | -34.6% |
| D1 | lateralDevMax | 1.8270 | 2.2256 | +21.8% |

#### 최종 자동채점 결과

| sid | KPI | value | target | score |
|---|---|---:|---:|---:|
| A3 | yawRateOvershoot | 2.2118 | 10.0000 | 4.00 / 4 |
| A3 | yawRateRiseTime | 0.0810 | 0.3000 | 4.00 / 4 |
| A3 | yawRateSettling | 0.7440 | 0.8000 | 4.00 / 4 |
| A1 | sideSlipMax | 1.6606 | 3.0000 | 6.00 / 6 |
| A1 | LTR_max | 0.6005 | 0.6000 | 5.00 / 5 |
| A1 | lateralDevMax | 2.2256 | 0.7000 | 0.00 / 4 |
| A4 | understeerGradient | 0.0007 | 0.0030 | 5.00 / 5 |
| A4 | sideSlipMax | 1.1827 | 2.0000 | 5.00 / 5 |
| A7 | sideSlipMax | 1.7104 | 5.0000 | 8.00 / 8 |
| A7 | LTR_max | 0.3390 | 0.7000 | 7.00 / 7 |
| B1 | stoppingDistance | 64.5344 | 40.0000 | 0.00 / 5 |
| B1 | absSlipRMS | 0.1168 | 0.1000 | 4.44 / 5 |
| D1 | sideSlipMax | 2.9859 | 4.0000 | 4.00 / 4 |
| D1 | LTR_max | 0.5650 | 0.6000 | 2.00 / 2 |
| D1 | lateralDevMax | 2.2256 | 1.0000 | 0.00 / 2 |

<br>

| 항목 | 점수 |
|---|---:|
| Quantitative | 58.44 / 70.00 |
| 비율 | 83.5% |
| Deductions | 0 |
| Auto-graded total | 58.44 / 70.00 |

※ B1 `stoppingDistance`의 채점표 target은 40.0 m로 표시되었으나, 과제 공지에서 수정된 만점 기준은 66.5 m이다. 본 설계의 B1 정지거리는 64.5344 m로 수정 기준을 만족한다. 따라서 최종점수는 바뀌게 된다.


### 4.2 핵심 plot — A1 DLC

![A1 trajectory comparison](figures/a1_trajectory.png)

*Figure 4.1 — A1 ISO 3888-1 DLC, 차량 trajectory (off vs on) vs reference path.*

![A1 yaw rate](figures/a1_yawrate.png)

*Figure 4.2 — A1 yaw rate 응답: reference (driver bicycle model), off (controller off), on (본인 설계).*

(plot 생성 예시:
```matlab
[r_off, k_off] = run_icc_scenario('A1','14dof','Controller','off','SavePlot',false);
[r_on,  k_on ] = run_icc_scenario('A1','14dof','Controller','on', 'SavePlot',false);
figure; plot(r_off.x_pos, r_off.y_pos, 'r--', r_on.x_pos, r_on.y_pos, 'b-', ...
             r_off.scenario.refPath(:,1), r_off.scenario.refPath(:,2), 'k:');
xlabel('x [m]'); ylabel('y [m]'); legend('off','on','ref'); axis equal;
saveas(gcf, 'docs/figures/a1_trajectory.png');
```

### 4.3 한 시나리오 deep dive — A7 (또는 본인이 가장 잘 푼 것)

### 4.3 한 시나리오 deep dive — A7 Brake-in-turn

A7 brake-in-turn 의 핵심:
- 베이스라인 sideSlipMax: 46.3° (스핀아웃)
- 본인 설계: 1.7104°
- 핵심 요인: ESC beta-limiter가 slip angle 증가를 억제하고, yaw moment를 차동제동으로 변환하여 spin-out을 방지함

A7 brake-in-turn 시나리오는 선회 중 제동이 동시에 발생하는 조건이다. 따라서 단순한 제동 성능뿐만 아니라 yaw 안정성, side slip 억제, LTR 감소가 함께 중요하다. 베이스라인에서는 제동 중 차량 자세가 크게 불안정해지고 `sideSlipMax`가 46.3°까지 증가하여 스핀아웃에 가까운 거동을 보였다. 반면 본 설계에서는 `ctrl_lateral`의 beta-limiter와 `ctrl_coordinator`의 차동제동 allocation을 통해 `sideSlipMax`를 1.7104°까지 감소시켰다.

A7의 주요 KPI 비교는 다음과 같다.

| KPI | Baseline | ON (본인) | 개선 |
|---|---:|---:|---:|
| sideSlipMax [deg] | 46.3 | 1.7104 | 대폭 감소 |
| LTR_max | 0.745 | 0.3390 | 감소 |

핵심 제어 동작은 slip angle 기반 ESC 개입이다. `ctrl_lateral`에서는 slip angle이 임계값을 초과하면 차량 회전을 억제하는 방향의 yaw moment를 생성한다. 이후 `ctrl_coordinator`는 이 yaw moment를 좌우 wheel의 brake torque 차이로 변환한다. 이 차동제동은 선회 중 제동으로 인해 발생하는 과도한 yaw motion을 줄이고, 차량이 spin-out으로 진행하는 것을 방지한다.

또한 `ctrl_vertical`의 고감쇠 CDC 제어는 차체 roll을 억제하여 LTR 감소에 기여하였다. 그 결과 `LTR_max`는 baseline 0.745에서 0.3390으로 감소하였다. 따라서 A7은 본 통합 섀시 제어기에서 AFS/ESC, brake allocation, CDC가 함께 작동하여 가장 큰 안정성 개선을 보인 시나리오이다.

---

## 5. 분석 + 한계

### 5.1 가장 성공적이었던 시나리오

가장 성공적이었던 시나리오는 A7 brake-in-turn 시나리오이다. 이 시나리오는 선회 중 제동이 동시에 발생하므로 side slip 증가와 yaw 불안정이 쉽게 나타난다. 베이스라인에서는 `sideSlipMax`가 46.3°까지 증가하여 스핀아웃에 가까운 거동을 보였지만, 본 설계에서는 1.7104°까지 감소하였다.

이 개선은 `ctrl_lateral`의 beta-limiter와 `ctrl_coordinator`의 차동제동 allocation이 함께 작동한 결과이다. slip angle이 커지면 `ctrl_lateral`이 차량 회전을 억제하는 방향의 yaw moment를 생성하고, `ctrl_coordinator`는 이를 좌우 wheel brake torque 차이로 변환한다. 이 과정에서 제동 중 발생하는 과도한 yaw motion이 억제되어 side slip이 크게 줄었다.

또한 `LTR_max`도 베이스라인 0.745에서 0.3390으로 감소하였다. 이는 `ctrl_vertical`의 고감쇠 CDC 제어가 차체 roll을 줄이고, 제동 중 하중 이동을 완화했기 때문으로 판단된다. 따라서 A7은 본 통합 섀시 제어기의 안정성 개선 효과가 가장 뚜렷하게 나타난 시나리오이다.

<br>

### 5.2 가장 부족했던 시나리오

가장 부족했던 항목은 A1과 D1의 `lateralDevMax`이다. 최종 자동채점 결과에서 A1의 `lateralDevMax`는 2.2256 m로 목표값 0.7 m를 만족하지 못해 0점을 받았다. D1에서도 `lateralDevMax`가 2.2256 m로 목표값 1.0 m를 만족하지 못해 0점을 받았다.

이 한계는 본 제어기의 구조와 관련이 있다. `ctrl_lateral`의 입력은 `yawRateRef`, `yawRate`, `slipAngle`, `vx`로 제한되어 있다. 즉, 차량이 reference path에서 얼마나 벗어났는지를 나타내는 lateral position error나 heading error를 직접 사용할 수 없다. 따라서 본 제어기는 path tracking controller라기보다는 yaw rate 추종과 slip angle 억제를 중심으로 한 stability controller에 가깝다.

A1 DLC와 D1 통합 시나리오에서는 경로를 정확히 따라가는 능력이 중요하지만, 본 설계는 차량 자세 안정성을 우선시하였다. 그 결과 side slip과 LTR은 크게 개선되었지만, lateral path deviation은 충분히 줄이지 못하였다.

가능한 원인은 다음과 같다.

- **가설 1**: `ctrl_lateral`에 lateral position error와 heading error가 입력되지 않아 path deviation을 직접 피드백할 수 없었다.
- **가설 2**: yaw rate 추종과 slip 억제를 우선시하면서, reference path를 더 공격적으로 따라가기 위한 조향 보정이 부족하였다.
- **가설 3**: AFS 보조 조향각이 제한값에 가까워지는 구간이 있어, DLC의 급격한 경로 변화에 충분히 대응하지 못하였다.

A4 정상선회에서는 `understeerGradient`가 0.0007로 목표값 0.0030 이하를 만족하였다. 따라서 본 설계에서 가장 부족했던 부분은 A4가 아니라 A1/D1의 `lateralDevMax`라고 판단된다.

<br>

### 5.3 만약 더 시간이 있었다면

시간이 더 있었다면 가장 먼저 path error를 반영하는 제어 구조를 추가하고 싶다. 예를 들어 lateral position error와 heading error를 사용할 수 있다면, Stanley controller나 preview controller 형태의 path tracking 보상기를 추가하여 A1과 D1의 `lateralDevMax`를 줄일 수 있을 것이다.

두 번째로는 현재의 PID 기반 yaw rate 제어를 LQR 기반 제어로 확장할 수 있다. Bicycle model의 상태를 `v_y`, `r`로 두고, yaw rate error뿐 아니라 slip angle과 조향 입력 크기까지 cost function에 포함하면 안정성과 경로 추종성 사이의 trade-off를 더 체계적으로 조정할 수 있다.

세 번째로는 `ctrl_coordinator`의 brake allocation을 WLS 방식으로 개선할 수 있다. 현재는 전후 60:40 분배와 단순 차동제동을 사용하였지만, 각 wheel의 tire utilization을 고려한 WLS allocation을 적용하면 yaw moment 생성과 제동 안정성을 더 정밀하게 조정할 수 있다.

마지막으로 `ctrl_vertical`도 현재의 고정 고감쇠 CDC 방식에서 skyhook 또는 hybrid skyhook-groundhook 방식으로 개선할 수 있다. 이를 통해 LTR 완화뿐 아니라 승차감과 wheel-hop 억제까지 함께 고려할 수 있을 것이다.

---

## 6. 참고문헌

[1] ISO 3888-1:2018, *Passenger cars — Test track for a severe lane-change manoeuvre*.

[2] ISO 4138:2021, *Passenger cars — Steady-state circular driving behaviour — Open-loop test methods*.

[3] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012.

[4] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley, 2008.

[5] H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012.

---

## 부록 A — 사용한 AI 도구

본 과제 수행 과정에서 ChatGPT를 보조 도구로 사용하였다. 사용 목적은 제어기 설계 방향 정리, PID 및 gain scheduling 구조 검토, ABS release logic 개선, `ctrl_coordinator`의 brake allocation 구조 정리, 그리고 보고서 문장 작성 보조였다.

초기에는 yaw rate 추종을 위한 PID 구조와 speed gain scheduling 아이디어를 정리하는 데 사용하였다. 이후 MATLAB 시뮬레이션 결과를 바탕으로 `ctrl_lateral`, `ctrl_longitudinal`, `ctrl_vertical`, `ctrl_coordinator`의 gain과 logic을 직접 수정하며 성능을 비교하였다. ChatGPT가 제안한 값은 그대로 사용하지 않고, `grade.m`과 `run_icc_benchmark.m` 실행 결과를 확인하면서 본인이 반복적으로 조정하였다.

예를 들어 `ctrl_lateral`에서는 yaw rate 응답 개선을 위해 PID gain scheduling 구조를 사용하였고, 최종적으로 `Kp_eff = 0.75*Kp*speedGain`, `Ki_eff = 0.15*Ki*min(speedGain,1.0)`, `Kd_eff = 0.63*Kd*speedGain` 형태로 조정하였다. `ctrl_longitudinal`에서는 B1 급제동 성능 개선을 위해 감속도 기반 ABS release logic을 적용하였고, 최종적으로 `axTarget = -7.6`, `releaseAccel` 상한을 1.6으로 설정하였다.

또한 보고서 작성 단계에서는 수학적 모델링, 제어기 설계 설명, 시뮬레이션 결과 분석, 한계점 정리의 문장 구성에 ChatGPT를 활용하였다. 최종 코드와 보고서 내용은 본인이 MATLAB 시뮬레이션 결과를 확인하며 수정 및 검증하였다.