% =========================================================================
% =============== Auto Hitch Project - Simulation Environment =============
% ====================== Ehsan Arabi - 04/30/2024 =========================
% =========================================================================
% Description:  This script simulates a hitching maneuver for testing
%               Kalman Filter and other path planning / control algorithms. 
% =========================================================================

clear; close all; clc;
addpath('./UtilityFunctions');

Vid_result_name = 'AH_2D_sim_'; % datestr(now, 'mm_dd_yyyy__HH_MM_SS')

rng(188);
%% Initializations
p = getParams();
RWA_max = p.RWA_max;
RWA_min = p.RWA_min;
Ts = p.Ts;
v = p.v;
x_bar = p.x_bar;
N = p.N;

% User Parameters
Show_RWA_on_video = 1;
Live_plot = 0;

% Approach Angle Case - Vehilce at 0 heading initially
trailer_angle = deg2rad(15); % trailer heading angle w.r.t host

% initial hitchball location
x_b = p.x_b;
y_b = p.y_b;
h_b = p.h_b;

% origin at true coupler location
x_c = p.x_c;
y_c = p.y_c;
h_c = p.h_c;

Vid_result_name = [Vid_result_name,'_Single_Shooting'];

Vid_result_name = [Vid_result_name,'_yb_',num2str(abs(y_b)),'_xb_',num2str(abs(x_b))];

delete(['.\Video_Results\',Vid_result_name,'.mp4']);

% Define the parameters for saving videos
frameRate = p.frameRate;
outputVideoFile = ['.\Video_Results\',Vid_result_name,'.mp4'];
videoWriter1 = VideoWriter(outputVideoFile, 'MPEG-4');
videoWriter1.Quality = 100;
videoWriter1.FrameRate = frameRate;open(videoWriter1);

outputVideoFile = ['.\Video_Results\',Vid_result_name,'estimation','.mp4'];
videoWriter2 = VideoWriter(outputVideoFile, 'MPEG-4');
videoWriter2.Quality = 100;
videoWriter2.FrameRate = frameRate;open(videoWriter2);

x_hist = [];
y_hist = [];
dx_hist = [];
dy_hist = [];
theta_hist= [];
dtheta_hist = [];
delta_hist =[];

accel = p.accel;
L = p.L;

% Cam coordinates : y direction is toward front of vehicle
%                   x direction toward left
%                   z toward ground (origin on ground, under camera)
% Vehicle coordinates : x direction is toward front of vehicle
%                       y direction toward left

t = 0;
theta_animation = 0;

x0 = zeros(N+1, 1);
dist = sqrt(x_b^2 + y_b^2);
x0(end) = dist/450;
cost = @(x) 0;

A = []; b = []; Aeq = []; beq = [];
lb = RWA_min*ones(N, 1);
ub = RWA_max*ones(N,1);

options = optimoptions("fmincon", "Display", "iter");
x_star = fmincon(cost, x0, A, b, Aeq, beq, lb, ub, @nonlcon, options);

t_star = linspace(0, x_star(end), N);
delta_star = x_star(1:end-1);
u = @(t) interp1(t_star, delta_star, t);
[tout, qout] = Simulator([t_star(1), t_star(end)], [x_b, y_b, 0], delta_star);

%%
for i = 1:length(qout)
    t=tout(i);
    x_b = qout(i, 1);
    y_b = qout(i, 2);
    theta = qout(i, 3);

    Hitchball_world = [x_b, y_b, -h_b]';
    Coupler_world = [x_c, y_c, -h_c]';

    Hitchball_world_hit(1:3,i) = Hitchball_world;
    True_coupler_location_hist(1:3,i) = Coupler_world;

    Coupler_world_noisy = [x_c, y_c, -h_c]';

    Noisy_coupler_location_hist(1:3,i) = Coupler_world_noisy;
    True_hitch_to_coupler(1:3,i)  = Hitchball_world - Coupler_world;
    Noisy_hitch_to_coupler(1:3,i) = Hitchball_world - Coupler_world_noisy;

    smoothedPoint = [Hitchball_world(1:2) - Coupler_world_noisy(1:2); Coupler_world_noisy(3)];

    Coupler_estim_loc = [Hitchball_world(1:2) - smoothedPoint(1:2);smoothedPoint(3)];
    Coupler_estim_loc_hist (1:3,i) = Coupler_estim_loc;

    x_f = x_c - smoothedPoint(1); % coupler - final location to steering - x_f and y_f
    y_f = y_c - smoothedPoint(2);

    x_f_unrotated_noise_free = -True_hitch_to_coupler(1,end);
    y_f_unrotated_noise_free = -True_hitch_to_coupler(2,end);

    x_f_unrotated = x_f;
    y_f_unrotated = y_f;

    %%  Delta calculation
    % x_f and y_f changes due to vehicle yaw motion
    M_Rot = [cos(-theta)  -sin(-theta)
        sin(-theta)   cos(theta)];
    xy_f_rotated = M_Rot*[x_f;y_f];
    x_f = xy_f_rotated(1);
    y_f = xy_f_rotated(2);

    delta = u(t);

    %%

    Delta_hist_deg(i,1) = delta*180/pi;
    delta_hist(i,1) = delta;

    if t==0
        initial_delta = delta;
    end

    w =2500;
    l = 5900;

    x_animation = -x_f_unrotated_noise_free;
    y_animation = -y_f_unrotated_noise_free;
    a=0;

    x_rear = x_animation + x_bar*cos(theta);
    y_rear = y_animation + x_bar*sin(theta);

    x_hist = [x_hist; x_animation];
    y_hist = [y_hist; y_animation];
    theta_hist = [theta_hist; theta];
end

%% video from frames
f1 = figure(1010);
set(f1,'Position',[0   100   800   800 ]);

subplot(4,1,4);
plot(Delta_hist_deg(1:end),'.-b','MarkerSize',3.5); hold on;
plot([1 length(Delta_hist_deg)],[RWA_max  RWA_max]*180/pi,'--r');
plot([1 length(Delta_hist_deg)],-[RWA_max  RWA_max]*180/pi,'--r');
xlabel('Frames');
ylabel('RWA [deg]');

True_coupler_location_hist_rel = True_coupler_location_hist-Hitchball_world_hit;
Noisy_coupler_location_hist_rel = Noisy_coupler_location_hist-Hitchball_world_hit;

%%
figure(1010);
for j = 1:i
    if j>1
        delete(timeline);
    end
    subplot(4,1,1:3);
    x_rear = x_hist(j) + x_bar*cos(theta_hist(j));
    y_rear = y_hist(j) + x_bar*sin(theta_hist(j));
    delta = Delta_hist_deg(j)*pi/180;

    Coupler_estim_loc = Coupler_estim_loc_hist (1:3,j);

    drawVehicle(x_rear, y_rear, theta_hist(j), delta, w, l, L, x_bar);
    drawTrailer_angle(0, 0,trailer_angle, w, l, x_bar);hold on;
    scatter(Noisy_coupler_location_hist(1,j), Noisy_coupler_location_hist(2,j),55,"filled",'o','MarkerFaceColor','m');

    hold on; plot(x_hist(1:j),y_hist(1:j),'r-.');
    delete(findall(gcf,'type','annotation'));

    % Plot future traj based on current steering
    % [traj_x,traj_y] = get_future_traj(x_hist(j),y_hist(j),theta_hist(j), v, delta_hist(j), L, a,x_bar,Ts);
    % plot(traj_x,traj_y,':k');
    % 
    % if delta>RWA_max || delta <-RWA_max
    %     rwa_color = 'red';
    % else
    %     rwa_color = 'black';
    % end
    % 
    % if Show_RWA_on_video==1
    %     A1 = annotation('textbox', [0.2, 0.73, 0.1, 0.1],  'String', {['rwa = ', num2str(floor(delta*180/pi*10)/10),' deg']}, ...
    %         'FontSize', 12,  'FontWeight', 'bold',  'Color', rwa_color, 'BackgroundColor', 'white',  'EdgeColor', 'black', ...
    %         'HorizontalAlignment', 'center',  'VerticalAlignment', 'middle');
    %     set(A1, 'horizontalAlignment', 'left');
    %     A2 = annotation('textbox', [0.2, 0.35, 0.1, 0.1],  'String', {['True Hitch2Coupler = [', num2str(round(x_hist(j))),',',num2str(round(y_hist(j))),'] mm'],...
    %         ['Coupler2TrueCoupler = [', num2str(round(Coupler_estim_loc(1))),',',num2str(round(Coupler_estim_loc(2))),'] mm']}, ...
    %         'FontSize', 12,  'FontWeight', 'bold',  'Color', 'k', 'BackgroundColor', 'white',  'EdgeColor', 'black', ...
    %         'HorizontalAlignment', 'center',  'VerticalAlignment', 'middle');
    %     set(A2, 'horizontalAlignment', 'left');
    % end

    if j==1
        axis auto;
        Axis = axis;
    end
    axis(Axis);
    hold off;

    subplot(4,1,4);
    timeline = plot([j j],[-90  90],'--k');

    drawnow;

    frame = getframe(gcf);
    writeVideo(videoWriter1, frame);
end

close(videoWriter1);