function drawVehicle(x, y, theta_rad, delta_rad, w, l, L, x_bar)
% Convert theta from degrees to radians
tire_l = 788;
tire_w = 203; %
%     x_bar = 1; % hitchball distance from rear axle

%     theta_rad = deg2rad(theta);
%     delta_rad = deg2rad(delta);
tireAngle = theta_rad + delta_rad;

% Calculate the center position of the vehicle
centerX = x + (L/2) * cos(theta_rad);
centerY = y + (L/2) * sin(theta_rad);

% Calculate front and rear axle positions
frontAxleX = x + L * cos(theta_rad);
frontAxleY = y + L * sin(theta_rad);

% Vehicle corner points calculation for the rectangle
frontLeft_ax = [frontAxleX - w/2*sin(theta_rad), frontAxleY + w/2*cos(theta_rad)];
frontRight_ax = [frontAxleX + w/2*sin(theta_rad), frontAxleY - w/2*cos(theta_rad)];
rearLeft_ax = [x - w/2*sin(theta_rad), y + w/2*cos(theta_rad)];
rearRight_ax = [x + w/2*sin(theta_rad), y - w/2*cos(theta_rad)];

% Vehicle corner points calculation
frontLeft = [centerX + l/2*cos(theta_rad) - w/2*sin(theta_rad), centerY + l/2*sin(theta_rad) + w/2*cos(theta_rad)];
frontRight = [centerX + l/2*cos(theta_rad) + w/2*sin(theta_rad), centerY + l/2*sin(theta_rad) - w/2*cos(theta_rad)];
rearLeft = [centerX - l/2*cos(theta_rad) - w/2*sin(theta_rad), centerY - l/2*sin(theta_rad) + w/2*cos(theta_rad)];
rearRight = [centerX - l/2*cos(theta_rad) + w/2*sin(theta_rad), centerY - l/2*sin(theta_rad) - w/2*cos(theta_rad)];

% Draw the vehicle rectangle
vehicleX = [frontLeft(1), frontRight(1), rearRight(1), rearLeft(1), frontLeft(1)];
vehicleY = [frontLeft(2), frontRight(2), rearRight(2), rearLeft(2), frontLeft(2)];
fill(vehicleX, vehicleY, [0.9 0.9 0.9]);
%     plot(vehicleX, vehicleY, 'b', 'LineWidth', 2); % Draw outline in blue
hold on;

% Draw axles - adjust lines to represent the axles accurately
% Front axle
plot([frontLeft_ax(1), frontRight_ax(1)], [frontLeft_ax(2), frontRight_ax(2)], 'k--', 'LineWidth', 2);
% Rear axle
plot([rearLeft_ax(1), rearRight_ax(1)], [rearLeft_ax(2), rearRight_ax(2)], 'k--', 'LineWidth', 2);

% Draw the tires using l and w
% FL
x_1 = frontLeft_ax(1)-tire_l/2*cos(tireAngle)-tire_w/2*sin(tireAngle);
y_1 = frontLeft_ax(2)-tire_l/2*sin(tireAngle)+tire_w/2*cos(tireAngle);
leftTireX = [x_1,  x_1+tire_l*cos(tireAngle),  x_1+tire_l*cos(tireAngle)+tire_w*sin(tireAngle), x_1+tire_w*sin(tireAngle), x_1];
leftTireY = [y_1,  y_1+tire_l*sin(tireAngle),  y_1+tire_l*sin(tireAngle)-tire_w*cos(tireAngle), y_1-tire_w*cos(tireAngle), y_1];
fill(leftTireX, leftTireY, [0 0 0]);

% FR
x_1 = frontRight_ax(1)-tire_l/2*cos(tireAngle)-tire_w/2*sin(tireAngle);
y_1 = frontRight_ax(2)-tire_l/2*sin(tireAngle)+tire_w/2*cos(tireAngle);
leftTireX = [x_1,  x_1+tire_l*cos(tireAngle),  x_1+tire_l*cos(tireAngle)+tire_w*sin(tireAngle), x_1+tire_w*sin(tireAngle), x_1];
leftTireY = [y_1,  y_1+tire_l*sin(tireAngle),  y_1+tire_l*sin(tireAngle)-tire_w*cos(tireAngle), y_1-tire_w*cos(tireAngle), y_1];
fill(leftTireX, leftTireY, [0 0 0]);

Rear_tireAngle=theta_rad;
% RL
x_1 = rearLeft_ax(1)-tire_l/2*cos(Rear_tireAngle)-tire_w/2*sin(Rear_tireAngle);
y_1 = rearLeft_ax(2)-tire_l/2*sin(Rear_tireAngle)+tire_w/2*cos(Rear_tireAngle);
leftTireX = [x_1,  x_1+tire_l*cos(Rear_tireAngle),  x_1+tire_l*cos(Rear_tireAngle)+tire_w*sin(Rear_tireAngle), x_1+tire_w*sin(Rear_tireAngle), x_1];
leftTireY = [y_1,  y_1+tire_l*sin(Rear_tireAngle),  y_1+tire_l*sin(Rear_tireAngle)-tire_w*cos(Rear_tireAngle), y_1-tire_w*cos(Rear_tireAngle), y_1];
fill(leftTireX, leftTireY, [0 0 0]);

% RR
x_1 = rearRight_ax(1)-tire_l/2*cos(Rear_tireAngle)-tire_w/2*sin(Rear_tireAngle);
y_1 = rearRight_ax(2)-tire_l/2*sin(Rear_tireAngle)+tire_w/2*cos(Rear_tireAngle);
leftTireX = [x_1,  x_1+tire_l*cos(Rear_tireAngle),  x_1+tire_l*cos(Rear_tireAngle)+tire_w*sin(Rear_tireAngle), x_1+tire_w*sin(Rear_tireAngle), x_1];
leftTireY = [y_1,  y_1+tire_l*sin(Rear_tireAngle),  y_1+tire_l*sin(Rear_tireAngle)-tire_w*cos(Rear_tireAngle), y_1-tire_w*cos(Rear_tireAngle), y_1];
fill(leftTireX, leftTireY, [0 0 0]);

% Hitch ball
x_b = x - x_bar*cos(theta_rad);
y_b = y - x_bar*sin(theta_rad);
line([x, x_b], [y, y_b], 'Color', 'k', 'LineWidth', 2);
scatter(x_b,y_b,55,"filled",'o','MarkerFaceColor','b');

% Adjust plot
axis equal;
grid on;
xlabel('X [mm]');
ylabel('Y [mm]');
title('Vehicle Position and Orientation');

hold off;
end
