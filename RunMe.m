%  MATLAB Source Codes for the publication Bai Li, and Youmin Zhang, "Fast
%  trajectory planning in Cartesian rather than Frenet frame: A precise
%  solution for autonomous driving in complex urban scenarios",
%  IFAC-PapersOnLine, XX(XX), XX-XX, 2020.
%  Copyright (C) 2020 Bai Li (libai@hnu.edu.cn, libaioutstanding@163.com)
%  2020.05.11
% ==============================================================================
clear all; close all; clc

% % Params w.r.t vehicle property
global params_
params_.vehicle_wheelbase = 2.8;
params_.vehicle_front_hang = 0.96;
params_.vehicle_rear_hang = 0.929;
params_.vehicle_width = 1.942;
params_.vehicle_length = params_.vehicle_wheelbase + params_.vehicle_front_hang + params_.vehicle_rear_hang;
params_.radius = hypot(0.25 * params_.vehicle_length, 0.5 * params_.vehicle_width);
params_.r2x = 0.25 * params_.vehicle_length - params_.vehicle_rear_hang;
params_.f2x = 0.75 * params_.vehicle_length - params_.vehicle_rear_hang;
params_.vehicle_v_max = 20.0;
params_.vehicle_a_max = 0.5;
params_.vehicle_phy_max = 0.7;
params_.vehicle_w_max = 0.5;

% % Params w.r.t DP search
global dp_
dp_.num_t_grids = 5;
dp_.num_s_grids = 7;
dp_.num_l_grids = 8;
dp_.unit_time = 2.0;
dp_.max_unit_s = dp_.unit_time * params_.vehicle_v_max;
dp_.min_unit_s = 0;
dp_.ds = linspace(dp_.min_unit_s, dp_.max_unit_s, dp_.num_s_grids);
dp_.dl = linspace(0, 1, dp_.num_l_grids);
dp_.w_collision = 1.0;
dp_.w_Ncollision = 10000;
dp_.w_Njerky = 10;
dp_.w_lat_change = 1.0;
dp_.w_lon_change = 1.0;
dp_.w_lon_achieved = 10.0;
dp_.w_biasd = 0.5;

% % Params w.r.t environment setups
global obstacles_ Nobs precise_timeline precise_timeline_index
Nobs = 5;
agv_vel = 10;

precise_timeline = [0 : 0.05 : (dp_.unit_time * dp_.num_t_grids)];
precise_timeline_index = cell(1, dp_.num_t_grids);
ind = round(linspace(1, length(precise_timeline), dp_.num_t_grids + 1));
for ii = 1 : dp_.num_t_grids
    elem.ind1 = ind(ii); elem.ind2 = ind(ii+1);
    precise_timeline_index{1,ii} = elem;
end
obstacles_ = GenerateObstacles(agv_vel);
global road_barriers_
road_barriers_ = GenerateRoadBarrierGrids();
global BV_ % Boundary values
BV_.s0 = 0;
BV_.l0 = 0.78;
[BV_.x0, BV_.y0, BV_.theta0] = ConvertFrenetToCartesian(BV_.s0, BV_.l0);
BV_.v0 = 20;
BV_.phy0 = 0.18;

% % Plan a coarse trajectory in Frenet frame via sample and DP search
[x, y, theta] = SearchDecisionTrajectoryViaDp();
DrawRoadScenario();
plot(x,y,'r'); hold on; drawnow;

% % Plan a precise trajectory in Cartesian frame via computational optimal control
cutting_rate = 0.8;
tf = dp_.unit_time * dp_.num_t_grids * cutting_rate;
temp = abs(precise_timeline - tf);
ind_end = find(temp == min(temp)); ind_end = ind_end(1);
Nfe = ind_end; x = x(1:Nfe); y = y(1:Nfe); theta = theta(1:Nfe);
[v, a, phy, w] = FormInitialGuess(x, y, theta, tf);

[~, ~, xr, yr, xf, yf] = SpecifyLocalBoxes(x, y, theta);
WriteInitialGuessForFirstTimeNLP(x, y, theta, xr, yr, xf, yf, v, a, phy, w);
WriteParameters(tf, Nfe, x(end), y(end), theta(end));
!ampl rr.run
[cur_x, cur_y, cur_theta, cur_infeasibility] = LoadStates();

iter = 0;
while (cur_infeasibility > 0.01)
    iter = iter + 1;
    if (iter > 6)
        error 'Failed _ 1';
    end
    SpecifyLocalBoxes(cur_x, cur_y, cur_theta);
    !ampl rr.run
    [cur_x, cur_y, cur_theta, cur_infeasibility] = LoadStates();
end

SpecifyLocalBoxes(cur_x, cur_y, cur_theta);
!ampl rr2.run
load opti_flag.txt
if (~opti_flag)
    error 'Failed _ 2';
end
[cur_x, cur_y, cur_theta, cur_infeasibility] = LoadStates();
plot(cur_x, cur_y, 'g');