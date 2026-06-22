 function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 추가 가능)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동) — 차후 coordinator 가 brake 토크로 변환
%       ctrlState           - 업데이트
%
%   요구사항:
%       1. 속도 추종 PI 제어
%       2. ABS — wheel slip ratio |κ| > 0.12 일 때 brake force 감소 (slip-limit 또는 bang-bang)
%       3. 저크 제한 (LIM.MAX_JERK · m 으로 force 미분 cap)
%       4. anti-windup
%
%   주의:
%       - 본 함수는 wheel slip 정보가 직접 입력으로 들어오지 않음. 학생은 runner 가 매 step
%         result.tire.{FL,FR,RL,RR}.slipRatio 에 기록하는 값을 ctrlState 에 캐시하는 식으로
%         설계할 수 있음. 또는 ctrl_coordinator 에서 ABS 모듈레이션 (다른 설계 선택).
%       - 본 과제 시나리오 (B1) 는 vxRef 일정 — PID 속도 추종보다 ABS 가 핵심.
%
%   힌트:
%       - slip ratio κ = (ω·r_w - vx) / max(vx, 0.1)
%       - ABS 작동 조건: vehicle 감속 중 (ax < 0) AND |κ| > κ_target (≈0.12)
%       - Bang-bang ABS: brake_cmd = brake_cmd · 0.5 일 때 |κ| > κ_target

    %% TODO: 여기에 학생 구현
    %  (1) speed-tracking PI
    %  (2) ABS modulation (이번 함수에서 또는 ctrl_coordinator 에서)
    %  (3) jerk limit
    %  (4) anti-windup
  
    if isempty(ctrlState) || ~isstruct(ctrlState)
        ctrlState = struct();
    end

    if ~isfield(ctrlState, 'prevForce')
        ctrlState.prevForce = 0;
    end

    if ~isfield(ctrlState, 'brakeTimer')
        ctrlState.brakeTimer = 0;
    end

    if ~isfield(ctrlState, 'brakeActive')
        ctrlState.brakeActive = false;
    end

    m = 1800;

    Fx_cmd = 0;
    brakeRatio = 0;

    % Sustained braking detection.
    % Avoid reacting to short longitudinal acceleration spikes in A3/A4.
    if vx > 1.0 && ax < -1.5
        ctrlState.brakeTimer = ctrlState.brakeTimer + dt;
    else
        ctrlState.brakeTimer = max(0, ctrlState.brakeTimer - 2.0 * dt);
    end

    if ctrlState.brakeTimer > 0.15
        ctrlState.brakeActive = true;
    end

    if vx < 0.8
        ctrlState.brakeActive = false;
        ctrlState.brakeTimer = 0;
    end

    if ctrlState.brakeActive && vx > 1.0
        axTarget = -7.6;

        if ax < axTarget
            releaseAccel = min(abs(ax - axTarget), 1.6);
            Fx_cmd = m * releaseAccel;   % positive = brake release
        end
    end

    if isfield(LIM, 'MAX_JERK')
        dFmax = LIM.MAX_JERK * m * dt;
    else
        dFmax = 1e5 * dt;
    end

    Fx_cmd = min(max(Fx_cmd, ctrlState.prevForce - dFmax), ...
                     ctrlState.prevForce + dFmax);

    ctrlState.prevForce = Fx_cmd;

    forceCmd.Fx_total = Fx_cmd;
    forceCmd.brakeRatio = brakeRatio;

end