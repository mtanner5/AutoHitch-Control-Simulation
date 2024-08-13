%%  bicycle kinematic model
% function [dx, dy, dv, dtheta] = bicycleModelDynamics_atHitch(theta, v, delta, L, a, x_bar)
function [dq] = dynamics(t, q, u)
% bicycleModelDynamics - Function to simulate bicycle model dynamics
%
% Inputs:
%   theta - Heading angle of the vehicle (radians)
%   v     - Velocity of the vehicle (meters/second)
%   delta - Steering angle (radians)
%   L     - Wheelbase of the vehicle (meters)
%   a     - acceleration
%   x_bar  - hitch ball distance from the rear axle
%
% Outputs:
%   dx      - Time derivative of global x position
%   dy      - Time derivative of global y position
%   dtheta  - Time derivative of heading angle
p = getParams();
L = p.L;
v = p.v;
x_bar = p.x_bar;

x = q(1, :);
y = q(2, :);
theta = q(3, :);

delta = u(1, :);

% Compute the bicycle model dynamics
S = L/tan(delta);
beta = atan(x_bar/S); % angle to hitch ball position

dx = v * cos(theta-beta);
dy = v * sin(theta-beta);
dtheta = v / L * tan(delta)*cos(beta);

dq = zeros(size(q));
dq(1, :) = dx;
dq(2, :) = dy;
dq(3, :) = dtheta;
end
% =========================================================================
% =========================================================================