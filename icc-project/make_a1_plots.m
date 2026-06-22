%% make_a1_plots.m
% A1 DLC plot 생성

clear; clc; close all;

% 프로젝트 초기화
run('scripts/utils/init_project.m');

% 그림 저장 폴더 만들기
figDir = fullfile(pwd, 'docs', 'figures');

if ~exist(figDir, 'dir')
    mkdir(figDir);
end

% A1 시나리오 controller off 실행
[r_off, k_off] = run_icc_scenario('A1', '14dof', ...
    'Controller', 'off', 'SavePlot', false);

% A1 시나리오 controller on 실행
[r_on, k_on] = run_icc_scenario('A1', '14dof', ...
    'Controller', 'on', 'SavePlot', false);

%% 1) A1 trajectory plot
figure;

plot(r_off.x_pos, r_off.y_pos, 'r--', 'LineWidth', 1.5);
hold on;

plot(r_on.x_pos, r_on.y_pos, 'b-', 'LineWidth', 1.5);

plot(r_off.scenario.refPath(:,1), r_off.scenario.refPath(:,2), ...
    'k:', 'LineWidth', 1.5);

xlabel('x [m]');
ylabel('y [m]');
title('A1 DLC Trajectory');
legend('Controller off', 'Controller on', 'Reference path', ...
    'Location', 'best');

axis equal;
grid on;

saveas(gcf, fullfile(figDir, 'a1_trajectory.png'));

%% 2) A1 yaw rate plot
figure;

plot(r_off.t, r_off.yawRate, 'r--', 'LineWidth', 1.5);
hold on;

plot(r_on.t, r_on.yawRate, 'b-', 'LineWidth', 1.5);

plot(r_on.t, r_on.yawRateRef, 'k:', 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Yaw rate [rad/s]');
title('A1 Yaw Rate Response');
legend('Controller off', 'Controller on', 'Yaw rate reference', ...
    'Location', 'best');

grid on;

saveas(gcf, fullfile(figDir, 'a1_yawrate.png'));

fprintf('\nA1 plot 저장 완료!\n');
fprintf('저장 위치: %s\n', figDir);