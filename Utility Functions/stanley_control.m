function delta_stan = stanley_control(Coupler_estim_loc,Hitchball_world,smoothedPoint,trailer_angle,theta_animation,RWA_max,v)
% Assuming the linear path out of coupler
Stanley_x_path = Coupler_estim_loc(1) + (0:100:15000) * cos(trailer_angle);
Stanley_y_path = Coupler_estim_loc(2) + (0:100:15000) * sin(trailer_angle);

% stanley uses distance from front axle for forward driving and
% rear axle for reverse.
% Rear_axle_x = Hitchball_world(1)+(x_bar)*cos(theta_animation);
% Rear_axle_y = Hitchball_world(2)+(x_bar)*sin(theta_animation);
% distances = sqrt((x_path - Rear_axle_x).^2 + (y_path - Rear_axle_y).^2);

% using hitchball position instead
distances = sqrt((Stanley_x_path - Hitchball_world(1)).^2 + (Stanley_y_path - Hitchball_world(2)).^2);

[minDistance, idx] = min(distances);
% Cross-track error
dx = Stanley_x_path(idx) - smoothedPoint(1);
dy = Stanley_y_path(idx) - smoothedPoint(2);
% Angle from vehicle to path point
angle_to_path = atan2(dy, dx);
% Vehicle heading
heading_error = angle_to_path - theta_animation;
heading_error = mod(heading_error + pi, 2*pi) - pi;
% Sign of cross-track error
if sin(heading_error) > 0
    e = -minDistance;
else
    e = +minDistance;
end

theta_e = theta_animation-trailer_angle;
% wrap to pi
theta_e = mod(theta_e + pi, 2*pi) - pi;
k = 0.25;

% Stanley control
delta_stan = theta_e + atan(k * e / v);
delta_stan = max(min(delta_stan, RWA_max), -RWA_max);
end