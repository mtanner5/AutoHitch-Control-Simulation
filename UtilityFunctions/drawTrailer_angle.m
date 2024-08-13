% =========================================================================
% =========================================================================
function drawTrailer_angle(x, y, trailer_angle, w, l, x_bar)
hold on;
% x,y are the coordinates of the coupler
% trailer_angle is the rotation angle of the trailer in degrees
% w is the width of the trailer
% l is the length of the trailer
% x_bar is the distance from the coupler to the trailer frame

% Convert angle to radians
theta = trailer_angle;

% Calculate the center position of the vehicle
centerX = x - x_bar - l/2;
centerY = y;

% Vehicle corner points calculation
frontLeft = [centerX + l/2, centerY + w/2];
frontRight = [centerX + l/2, centerY - w/2];
rearLeft = [centerX - l/2, centerY + w/2];
rearRight = [centerX - l/2, centerY - w/2];

% Rotate each corner point around the center
rotatedFrontLeft = rotatePoint(frontLeft, 0, 0, theta);
rotatedFrontRight = rotatePoint(frontRight, 0, 0, theta);
rotatedRearLeft = rotatePoint(rearLeft, 0, 0, theta);
rotatedRearRight = rotatePoint(rearRight, 0, 0, theta);

% Draw lines from the coupler to the front corners
plot([rotatedFrontLeft(1), x], [rotatedFrontLeft(2), y], '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
plot([rotatedFrontRight(1), x], [rotatedFrontRight(2), y], '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 2);

% Draw the vehicle rectangle
TrailerX = [rotatedFrontLeft(1), rotatedFrontRight(1), rotatedRearRight(1), rotatedRearLeft(1), rotatedFrontLeft(1)];
TrailerY = [rotatedFrontLeft(2), rotatedFrontRight(2), rotatedRearRight(2), rotatedRearLeft(2), rotatedFrontLeft(2)];
fill(TrailerX, TrailerY, [0.95 0.95 0.95]);

hold off;
end

function [rotatedPoint] = rotatePoint(point, cx, cy, angle)
% This function rotates a point around a given center by angle radians
dx = point(1) - cx;
dy = point(2) - cy;

rotatedX = cos(angle) * dx - sin(angle) * dy + cx;
rotatedY = sin(angle) * dx + cos(angle) * dy + cy;

rotatedPoint = [rotatedX, rotatedY];
end
