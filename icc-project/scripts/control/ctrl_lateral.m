function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad], 부호 driver delta 와 동일 방향
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (ctrl_coordinator 가 brake 차동으로 변환)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향 (예: PID, LQR, pole placement, SMC 중 택일)
%       2. |slipAngle| > β_threshold 일 때 yaw moment 인가 (driver intent 와 반대 방향)
%       3. vx 적응 — 저속/고속 게인 differential (예: gain scheduling, LPV)
%       4. anti-windup, saturation 처리
%
%   금지:
%       - scenario id 분기 (예: 'A1 이면 X' 같은 hardcoding)
%       - LIM.MAX_STEER_ANGLE 위반
%       - global 변수 사용
%
%   힌트:
%       - PID 출발점은 sim_params.m 의 CTRL.LAT.Kp/Ki/Kd 값
%       - LQR 설계 시 Bicycle Model state-space (scripts/control/calc_bicycle_model.m 참조)
%       - β-limiter 는 다음 형태가 일반적:
%             if |β| > β_th
%                 M_z = -K_β · sign(β) · (|β| - β_th) · f(vx)
%       - speed scheduling: f(vx) = min(vx/v_ref, 2)

    %% TODO: 여기에 학생 구현 작성
    %  (1) PID/LQR/... 으로 yaw rate 추종 보조 조향 계산
    %  (2) slip angle 임계 초과 시 yaw moment 계산
    %  (3) speed scheduling 적용
    %  (4) limit/saturation
%CTRL_LATERAL 횡방향 통합 제어기 (AFS + ESC)
% yaw rate 추종 AFS + slip angle 제한 ESC + speed scheduling

    % 0. State initialization
    if isempty(ctrlState) || ~isstruct(ctrlState)
        ctrlState = struct();
    end

    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
    end

    if ~isfield(ctrlState, 'prevError')
        ctrlState.prevError = 0;
    end

    if ~isfield(ctrlState, 'dFilt')
        ctrlState.dFilt = 0;
    end

    % 1. Gain
    Kp = CTRL.LAT.Kp;
    Ki = CTRL.LAT.Ki;
    Kd = CTRL.LAT.Kd;

    if isfield(CTRL.LAT, 'intMax')
        intMax = CTRL.LAT.intMax;
    else
        intMax = 0.5;
    end

    % 2. Speed scheduling
    v_ref = 20;
    speedGain = min(max(vx / v_ref, 0.3), 1.5);

    % 3. Yaw rate error
    error = yawRateRef - yawRate;

    steadyCorner = abs(error) < 0.03;

    Kp_eff = 0.75 * Kp * speedGain;
    Ki_eff = 0.15 * Ki * min(speedGain, 1.0);
    Kd_eff = 0.63 * Kd * speedGain;

    if steadyCorner
        Kp_eff = 0.35 * Kp_eff;
        Ki_eff = 0.0;
        Kd_eff = 0.35 * Kd_eff;
    end

    % 4. Anti-windup integral
    ctrlState.intError = ctrlState.intError + error * dt;
    ctrlState.intError = min(max(ctrlState.intError, -intMax), intMax);

    % 5. Derivative filter
    dErrorRaw = (error - ctrlState.prevError) / max(dt, 1e-4);

    alpha = 0.80;
    ctrlState.dFilt = alpha * ctrlState.dFilt + (1 - alpha) * dErrorRaw;
    dError = ctrlState.dFilt;

    % 6. AFS steering command
    steerCmd = Kp_eff * error ...
             + Ki_eff * ctrlState.intError ...
             + Kd_eff * dError;


if abs(yawRateRef) > 0.12 && abs(error) > 0.02 && abs(slipAngle) < deg2rad(4.0)
    steerCmd = steerCmd + 0.040 * sign(error);
end
    ctrlState.prevError = error;

    % 7. Steering saturation
    if isfield(LIM, 'MAX_STEER_ANGLE')
        maxSteer = LIM.MAX_STEER_ANGLE;
    else
        maxSteer = deg2rad(30);
    end

    steerCmd = min(max(steerCmd, -maxSteer), maxSteer);

    % 8. ESC beta limiter
    beta_th = deg2rad(4.5);
    yawMoment = 0;

    if abs(slipAngle) > beta_th
        beta_excess = abs(slipAngle) - beta_th;
        Kbeta = 350;

        yawMoment = -Kbeta * sign(slipAngle) ...
                    * beta_excess * min(speedGain, 1.1);
    end

    % 9. Yaw moment saturation
    if isfield(LIM, 'MAX_YAW_MOMENT')
        maxYawMoment = LIM.MAX_YAW_MOMENT;
    else
        maxYawMoment = 4000;
    end

    yawMoment = min(max(yawMoment, -maxYawMoment), maxYawMoment);

    % 10. Output
    deltaAdd.steerAngle = steerCmd;
    deltaAdd.yawMoment  = yawMoment;

end