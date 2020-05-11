function [v, a, phy, w] = FormInitialGuess(x, y, theta, tf)
Nfe = length(x);
v = zeros(1, Nfe);
a = zeros(1, Nfe);
dt = tf / (Nfe - 1);
for ii = 2 : Nfe
    v(ii) = sqrt(((x(ii) - x(ii-1)) / dt)^2 + ((y(ii) - y(ii-1)) / dt)^2);
end
for ii = 2 : Nfe
    a(ii) = (v(ii) - v(ii-1)) / dt;
end
phy = zeros(1, Nfe);
w = zeros(1, Nfe);
global params_
wb = params_.vehicle_wheelbase;
for ii = 2 : (Nfe-1)
    phy(ii) = atan((theta(ii+1) - theta(ii)) * wb / (dt * v(ii)));
end
ind = find(phy > params_.vehicle_phy_max); phy(ind) = params_.vehicle_phy_max;
ind = find(phy < -params_.vehicle_phy_max); phy(ind) = -params_.vehicle_phy_max;

for ii = 2 : (Nfe-1)
    w(ii) = (phy(ii+1) - phy(ii)) / dt;
end
ind = find(w > params_.vehicle_w_max); w(ind) = params_.vehicle_w_max;
ind = find(w < -params_.vehicle_w_max); w(ind) = -params_.vehicle_w_max;
end