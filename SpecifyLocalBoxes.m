function [BVr, BVf, xr, yr, xf, yf] = SpecifyLocalBoxes(x, y, theta)
warning off

NE = length(x);
cos_theta = cos(theta);
sin_theta = sin(theta);
global params_
xr = x + params_.r2x .* cos_theta;
yr = y + params_.r2x .* sin_theta;
xf = x + params_.f2x .* cos_theta;
yf = y + params_.f2x .* sin_theta;
BVr = zeros(NE,4);
BVf = zeros(NE,4); % xmin, xmax, ymin, ymax

for ii = 1 : NE
    lb = GetBoxVertexes(xr(ii), yr(ii), ii);
    if (~any(lb))
        counter = 0;
        is_lb_nonzero = 0;
        while (~is_lb_nonzero)
            counter = counter + 1;
            for jj = 1 : 4
                switch jj
                    case 1
                        x_nudge = xr(ii) + counter * 0.05;
                        y_nudge = yr(ii);
                    case 2
                        x_nudge = xr(ii) - counter * 0.05;
                        y_nudge = yr(ii);
                    case 3
                        x_nudge = xr(ii);
                        y_nudge = yr(ii) + counter * 0.05;
                    case 4
                        x_nudge = xr(ii);
                        y_nudge = yr(ii) - counter * 0.05;
                end
                lb = GetBoxVertexes(x_nudge, y_nudge, ii);
                if (any(lb))
                    is_lb_nonzero = 1;
                    x = x_nudge;
                    y = y_nudge;
                    break;
                end
            end
        end
    end
    BVr(ii,:) = [xr(ii) - lb(2), xr(ii) + lb(4), yr(ii) - lb(3), yr(ii) + lb(1)];
    
    lb = GetBoxVertexes(xf(ii), yf(ii), ii);
    if (~any(lb))
        counter = 0;
        is_lb_nonzero = 0;
        while (~is_lb_nonzero)
            counter = counter + 1;
            for jj = 1 : 4
                switch jj
                    case 1
                        x_nudge = xf(ii) + counter * 0.05;
                        y_nudge = yf(ii);
                    case 2
                        x_nudge = xf(ii) - counter * 0.05;
                        y_nudge = yf(ii);
                    case 3
                        x_nudge = xf(ii);
                        y_nudge = yf(ii) + counter * 0.05;
                    case 4
                        x_nudge = xf(ii);
                        y_nudge = yf(ii) - counter * 0.05;
                end
                lb = GetBoxVertexes(x_nudge, y_nudge, ii);
                if (any(lb))
                    is_lb_nonzero = 1;
                    x = x_nudge;
                    y = y_nudge;
                    break;
                end
            end
        end
    end
    BVf(ii,:) = [xf(ii) - lb(2), xf(ii) + lb(4), yf(ii) - lb(3), yf(ii) + lb(1)];
end

delete('CC');
fid = fopen('CC', 'w');
for ii = 1 : NE
    fprintf(fid, '%g 1 %f \r\n', ii, BVr(ii,1));
    fprintf(fid, '%g 2 %f \r\n', ii, BVr(ii,2));
    fprintf(fid, '%g 3 %f \r\n', ii, BVr(ii,3));
    fprintf(fid, '%g 4 %f \r\n', ii, BVr(ii,4));
    fprintf(fid, '%g 5 %f \r\n', ii, BVf(ii,1));
    fprintf(fid, '%g 6 %f \r\n', ii, BVf(ii,2));
    fprintf(fid, '%g 7 %f \r\n', ii, BVf(ii,3));
    fprintf(fid, '%g 8 %f \r\n', ii, BVf(ii,4));
end
fclose(fid);
end

function lb = GetBoxVertexes(x, y, time_index)
global params_
% up left down right
basic_step = 0.2;
max_step = 2.0;
lb = [1 1 1 1] .* params_.radius;
if (~IsBoxValid(x,y,time_index,lb))
    lb = zeros(1,4);
    return;
end
is_completed = zeros(1,4);
while (sum(is_completed) < 4)
    for ind = 1 : 4
        if (is_completed(ind))
            continue;
        end
        test = lb;
        if (test(ind) + basic_step > max_step)
            is_completed(ind) = 1;
            continue;
        end
        test(ind) = test(ind) + basic_step;
        if (IsCurrentEnlargementValid(x, y, test, lb, ind, time_index))
            lb = test;
        else
            is_completed(ind) = 1;
        end
    end
end
lb = lb - params_.radius;
end

function is_valid = IsCurrentEnlargementValid(x, y, test, lb, ind, time_index)
is_valid = 0;
switch ind
    case 1
        V_check = [x - lb(2), y + lb(1); x + lb(4), y + lb(1); x + test(4), y + test(1); x - test(2), y + test(1)];
    case 2
        V_check = [x - lb(2), y + lb(1); x - lb(2), y - lb(3); x - test(2), y - test(3); x - test(2), y + test(1)];
    case 3
        V_check = [x + lb(4), y - lb(3); x - lb(2), y - lb(3); x - test(2), y - test(3); x + test(4), y - test(3)];
    case 4
        V_check = [x + lb(4), y - lb(3); x + lb(4), y + lb(1); x + test(4), y + test(1); x + test(4), y - test(3)];
    otherwise
        return;
end

global road_barriers_
ind = find(abs(road_barriers_.x - x) + abs(road_barriers_.y - y) <= 15);
if (any(inpolygon(road_barriers_.x(ind), road_barriers_.y(ind), V_check(:,1)', V_check(:,2)')))
    return;
end

Vego = CreateVehicleEdgeGrids(V_check);
global obstacles_
for ii = 1 : size(obstacles_,2)
    cur_obs_x = obstacles_{1,ii}.x(time_index);
    cur_obs_y = obstacles_{1,ii}.y(time_index);
    if (abs(cur_obs_x - x) + abs(cur_obs_y - y) >= 20)
        continue;
    end
    cur_obs_theta = obstacles_{1,ii}.theta(time_index);
    V = CreateVehicleCornerPoints(cur_obs_x, cur_obs_y, cur_obs_theta);
    if (any(inpolygon(Vego.x, Vego.y, V.x, V.y)))
        return;
    end
end
is_valid = 1;
end

function V = CreateVehicleCornerPoints(x, y, theta)
global params_
cos_theta = cos(theta);
sin_theta = sin(theta);
vehicle_half_width = params_.vehicle_width * 0.5;
AX = x + (params_.vehicle_front_hang + params_.vehicle_wheelbase) * cos_theta - vehicle_half_width * sin_theta;
BX = x + (params_.vehicle_front_hang + params_.vehicle_wheelbase) * cos_theta + vehicle_half_width * sin_theta;
CX = x - params_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta;
DX = x - params_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta;
AY = y + (params_.vehicle_front_hang + params_.vehicle_wheelbase) * sin_theta + vehicle_half_width * cos_theta;
BY = y + (params_.vehicle_front_hang + params_.vehicle_wheelbase) * sin_theta - vehicle_half_width * cos_theta;
CY = y - params_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta;
DY = y - params_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta;
V.x = [AX, BX, CX, DX, AX];
V.y = [AY, BY, CY, DY, AY];
end

function VV = CreateVehicleEdgeGrids(V)
AX = V(1,1);
BX = V(2,1);
CX = V(3,1);
DX = V(4,1);
AY = V(1,2);
BY = V(2,2);
CY = V(3,2);
DY = V(4,2);
VV.x = [linspace(AX, BX, 4), linspace(BX, CX, 4), linspace(CX, DX, 4), linspace(DX, AX, 4)];
VV.y = [linspace(AY, BY, 4), linspace(BY, CY, 4), linspace(CY, DY, 4), linspace(DY, AY, 4)];
end

function is_valid = IsBoxValid(x, y, time_index, lb)
is_valid = 0;
C = [x + lb(4), y - lb(3)];
D = [x - lb(2), y - lb(3)];
A = [x - lb(2), y + lb(1)];
B = [x + lb(4), y + lb(1)];
V_check = [A(1), A(2); B(1), B(2); C(1), C(2); D(1), D(2)];

global road_barriers_
ind = find(abs(road_barriers_.x - x) + abs(road_barriers_.y - y) <= 10);
if (any(inpolygon(road_barriers_.x(ind), road_barriers_.y(ind), V_check(:,1)', V_check(:,2)')))
    return;
end

Vego = CreateVehicleEdgeGrids(V_check);
global obstacles_
for ii = 1 : size(obstacles_,2)
    cur_obs_x = obstacles_{1,ii}.x(time_index);
    cur_obs_y = obstacles_{1,ii}.y(time_index);
    if (abs(cur_obs_x - x) + abs(cur_obs_y - y) >= 10)
        continue;
    end
    cur_obs_theta = obstacles_{1,ii}.theta(time_index);
    V = CreateVehicleCornerPoints(cur_obs_x, cur_obs_y, cur_obs_theta);
    if (any(inpolygon(Vego.x, Vego.y, V.x, V.y)))
        return;
    end
end
is_valid = 1;
end