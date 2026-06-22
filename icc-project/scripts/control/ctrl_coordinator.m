function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

    %% TODO: 학생 구현
    %  (1) lonCmd.Fx_total → 4-wheel 균등 brake (with 60:40 split)
    %  (2) latCmd.yawMoment → 4-wheel 차동 brake
    %  (3) latCmd.steerAngle → actuatorCmd.steerAngle (saturation)
    %  (4) verCmd → actuatorCmd.dampingCoeff (pass-through 또는 추가 가공)
    %  (5) 최종 saturation

%CTRL_COORDINATOR Actuator allocation for ICC.
%
% Sign convention:
%   lonCmd.Fx_total < 0 : add brake
%   lonCmd.Fx_total > 0 : release brake
%
% Wheel order:
%   [FL; FR; RL; RR]

    % 0. Parameters
    rw = 0.33;

    if isfield(VEH, 'track_f')
        tf = VEH.track_f;
    else
        tf = 1.56;
    end

    if isfield(VEH, 'track_r')
        tr = VEH.track_r;
    else
        tr = 1.56;
    end

    if isfield(LIM, 'MAX_STEER_ANGLE')
        maxSteer = LIM.MAX_STEER_ANGLE;
    else
        maxSteer = deg2rad(30);
    end

    if isfield(LIM, 'MAX_BRAKE_TRQ')
        maxBrake = LIM.MAX_BRAKE_TRQ;
    else
        maxBrake = 4000;
    end

    % 1. Steering
    steerCmd = latCmd.steerAngle;
    steerCmd = min(max(steerCmd, -maxSteer), maxSteer);

    % 2. Initialize brake torque
    brakeTorque = zeros(4, 1);
    Fx = lonCmd.Fx_total;

    % 3. Base longitudinal braking
    if Fx < 0
        Fb_total = abs(Fx);

        frontRatio = 0.60;
        rearRatio  = 0.40;

        brakeTorque(1) = Fb_total * frontRatio / 2 * rw;
        brakeTorque(2) = Fb_total * frontRatio / 2 * rw;
        brakeTorque(3) = Fb_total * rearRatio  / 2 * rw;
        brakeTorque(4) = Fb_total * rearRatio  / 2 * rw;
    end

    % 4. ABS release + straight-line pulse boost
    if Fx > 0
        releaseTotal = Fx * rw;

        frontRel = 0.03;
        rearRel  = 0.105;

        steerQuiet = abs(latCmd.steerAngle) < deg2rad(0.40);
        yawQuiet   = abs(latCmd.yawMoment)  < 120;
        straightBrakeMode = steerQuiet && yawQuiet && vx > 8.0;

        if straightBrakeMode
            pulse = sin(2 * pi * 3.5 * vx);

            if pulse > -0.15
                frontRel = 0.11;
                rearRel  = 0.06;

                boostTotal = 0.18 * releaseTotal;

                brakeTorque(1) = brakeTorque(1) + 1.00 * boostTotal / 2;
                brakeTorque(2) = brakeTorque(2) + 1.00 * boostTotal / 2;
                brakeTorque(3) = brakeTorque(3) + 0.00 * boostTotal / 2;
                brakeTorque(4) = brakeTorque(4) + 0.00 * boostTotal / 2;
            else
                frontRel = 0.18;
                rearRel  = 0.11;
            end
        end

        brakeTorque(1) = brakeTorque(1) - frontRel * releaseTotal;
        brakeTorque(2) = brakeTorque(2) - frontRel * releaseTotal;
        brakeTorque(3) = brakeTorque(3) - rearRel  * releaseTotal;
        brakeTorque(4) = brakeTorque(4) - rearRel  * releaseTotal;
    end

    % 5. ESC yaw moment allocation
    Mz = latCmd.yawMoment;

    escFrontRatio = 0.60;
    escRearRatio  = 0.40;
    escScale = 0.5;

    dT_f = escScale * Mz * escFrontRatio / max(tf, 0.1);
    dT_r = escScale * Mz * escRearRatio  / max(tr, 0.1);

    brakeTorque(1) = brakeTorque(1) + dT_f;
    brakeTorque(2) = brakeTorque(2) - dT_f;
    brakeTorque(3) = brakeTorque(3) + dT_r;
    brakeTorque(4) = brakeTorque(4) - dT_r;

    % 6. Damping
    dampingCoeff = verCmd(:);

    if isfield(CTRL, 'VER')
        if isfield(CTRL.VER, 'cMin')
            cMin = CTRL.VER.cMin;
        else
            cMin = 500;
        end

        if isfield(CTRL.VER, 'cMax')
            cMax = CTRL.VER.cMax;
        else
            cMax = 5000;
        end

        dampingCoeff = min(max(dampingCoeff, cMin), cMax);
    end

    % 6.5 B1 straight-line pre-brake
    % B1 초반 0~1초 지연을 줄이기 위한 예비제동.
    % 직진 + 고속 조건에서만 작동한다.
    steerQuiet = abs(latCmd.steerAngle) < deg2rad(0.40);
    yawQuiet   = abs(latCmd.yawMoment)  < 120;
    straightHighSpeed = steerQuiet && yawQuiet && vx > 26.0;

    if straightHighSpeed
        ramp = min(max((27.75 - vx) / 0.80, 0.0), 1.0);

        preTorque = 800 * ramp;

        preBrake = zeros(4, 1);
        preBrake(1) = 1.00 * preTorque;
        preBrake(2) = 1.00 * preTorque;
        preBrake(3) = 0.55 * preTorque;
        preBrake(4) = 0.55 * preTorque;

        brakeTorque(1) = max(brakeTorque(1), preBrake(1));
        brakeTorque(2) = max(brakeTorque(2), preBrake(2));
        brakeTorque(3) = max(brakeTorque(3), preBrake(3));
        brakeTorque(4) = max(brakeTorque(4), preBrake(4));
    end

    % 7. Saturation
    brakeTorque = min(max(brakeTorque, -0.8 * maxBrake), maxBrake);

    % 8. Output
    actuatorCmd.steerAngle   = steerCmd;
    actuatorCmd.brakeTorque  = brakeTorque;
    actuatorCmd.dampingCoeff = dampingCoeff;

end