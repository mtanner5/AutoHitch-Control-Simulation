function p = getParams()

% Vehicle parameters
p.RWA_max = 30*pi/180;  % Steering Saturation
p.RWA_min = -p.RWA_max;
p.v = -450;
p.accel = 0;
p.L = 3.6*1000;
p.x_bar = 1420; % [mm]   hitch ball distance from rear axle ;

% Path planning parameters
p.Ts = 0.1;

% Video parameters
p.frameRate = 15; % Frames per second

% Single shooting parameters
p.N = 15;

% initial hitchball location
p.x_b = 14e3;
p.y_b = 8e3;
p.h_b = -0.500e3;

% origin at true coupler location
p.x_c = 0;
p.y_c = 0;
p.h_c = -0.550e3;