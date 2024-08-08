% =========================================================================
% =============== Auto Hitch Project - Simulation Environment =============
% ====================== Ehsan Arabi - 04/30/2024 =========================
% =========================================================================
% Description:  This script simulates a hitching maneuver for testing
%               Kalman Filter and other path planning / control algorithms. 
% =========================================================================

clear all;close all; clc;
addpath('./UtilityFunctions');

Vid_result_name = ['AH_2D_sim_']; % datestr(now, 'mm_dd_yyyy__HH_MM_SS')

rng(188);
%% Initializations
% User Parameters
RWA_max           = 30*pi/180;  % Steering Saturation
Show_RWA_on_video = 1;
Live_plot = 0;

% Approach Angle Case - Vehilce at 0 heading initially
trailer_angle = deg2rad(15); % trailer heading angle w.r.t host
Stanley_Approach = 1;

Variable_Steer            = 1; % flag to steer larger initially or not
Variable_Steer_Gain_Decay = 1;
Noise_active              = 1;
Use_Kalman                = 1;

Noise_gain = 50;

Ts = 0.1;
v = -450;

% initial hitchball location
x_b = 14e3;
y_b = 0; 3e3;
h_b = -0.500e3;

% origin at true coupler location
x_c = 0;
y_c = 0;
h_c = -0.550e3;

if Variable_Steer==1
    Vid_result_name = [Vid_result_name,'_Variable_Steer'];
else
    Vid_result_name = [Vid_result_name,'_Constant_Steer'];
end
if Noise_active==1
    Vid_result_name = [Vid_result_name,'_Noisy_lvl_',num2str(Noise_gain)];
    if Use_Kalman==1
        Vid_result_name = [Vid_result_name,'_Kalman'];
    end
end

if Stanley_Approach==1
    Vid_result_name = [Vid_result_name,'_Stanley_',num2str(rad2deg(trailer_angle)),'deg'];
end

Vid_result_name = [Vid_result_name,'_yb_',num2str(abs(y_b)),'_xb_',num2str(abs(x_b))];


delete(['.\Video_Results\',Vid_result_name,'.mp4']);

% Define the parameters for saving videos
frameRate = 15; % Frames per second
outputVideoFile = ['.\Video_Results\',Vid_result_name,'.mp4'];
videoWriter1 = VideoWriter(outputVideoFile, 'MPEG-4');
videoWriter1.Quality = 100;
videoWriter1.FrameRate = frameRate;open(videoWriter1);

outputVideoFile = ['.\Video_Results\',Vid_result_name,'estimation','.mp4'];
videoWriter2 = VideoWriter(outputVideoFile, 'MPEG-4');
videoWriter2.Quality = 100;
videoWriter2.FrameRate = frameRate;open(videoWriter2);


x_hist =[];
y_hist =[];
dx_hist =[];
dy_hist =[];
theta_hist=[];
delta_hist =[];

hitched = 0;
accel = 0;
L = 3.6*1000;


% Cam coordinates : y direction is toward front of vehicle
%                   x direction toward left
%                   z toward ground (origin on ground, under camera)
% Vehicle coordinates : x direction is toward front of vehicle
%                       y direction toward left

t = 0;
i = 1;
theta_animation =0;

% Kalman filter initialization
% State vector: [x, y, z, dx, dy, dz]
x_k = [];
P_k = eye(5); % Initial state covariance matrix

% State transition matrix (adjusted per iteration)
Q = 0.05*diag(Noise_gain*[0.0333    0.0333    0.0333    0.0007    0.0007]); % Process noise covariance matrix
R = diag(Noise_gain^2/12*[1, 1, 1]); % Measurement noise covariance

while hitched<=1
    if ~isempty(dx_hist) && ~isempty(dy_hist)
        % moving coupler coordinate by the amount vehicle moved
        x_b = x_b+dx_hist(end);
        y_b = y_b+dy_hist(end);
    end

    Hitchball_world = [x_b, y_b, -h_b]';
    Coupler_world = [x_c, y_c, -h_c]';

    Hitchball_world_hit(1:3,i) = Hitchball_world;
    True_coupler_location_hist(1:3,i) = Coupler_world;

    Added_Noise = Noise_active*Noise_gain*randn(3,1);
    Coupler_world_noisy = [x_c, y_c, -h_c]' + Added_Noise;

    Noisy_coupler_location_hist(1:3,i) = Coupler_world_noisy;
    True_hitch_to_coupler(1:3,i)  = Hitchball_world - Coupler_world;
    Noisy_hitch_to_coupler(1:3,i) = Hitchball_world - Coupler_world_noisy;

    if Use_Kalman==1 && hitched==0
        A = [1 0 0 1 0;
             0 1 0 0 1;
             0 0 1 0 0;
             0 0 0 0 0;
             0 0 0 0 0];
        B = diag([0;0;0;1;1]);

        Current_p = [Hitchball_world(1:2) - Coupler_world_noisy(1:2); Coupler_world_noisy(3)];

        if isempty(x_k)
            x_k = [Current_p;0;0];
            u =zeros(5,1);
        else
            u =  [0;0;0;dx_hist(end);dy_hist(end)];
        end

        % Kalman filter prediction and update
        [x_k, P_k] = kalman_filter_step(x_k, P_k, A, Q, Current_p, R,B*u);

        % Save estimated position
        smoothedPoint = x_k(1:3);
        Estimated_coupler(1:3,i) =  smoothedPoint';

    else
        smoothedPoint = [Hitchball_world(1:2) - Coupler_world_noisy(1:2); Coupler_world_noisy(3)];
    end

    if hitched==0
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
        M_Rot = [cos(-theta_animation)  -sin(-theta_animation)
            sin(-theta_animation)   cos(theta_animation)];
        xy_f_rotated = M_Rot*[x_f;y_f];
        x_f = xy_f_rotated(1);
        y_f = xy_f_rotated(2);

        x_bar = 1420; % [mm]   hitch ball distance from rear axle ;
        beta = atan(2*x_bar*y_f/(x_f^2+y_f^2-2*x_bar*x_f));
        if abs(x_f)>=10 || abs(y_f)>=10
            delta = atan(2*L*y_f/(x_f^2+y_f^2-2*x_bar*x_f));
        else
            delta = Delta_hist_deg(i-1)*pi/180;
        end
        T = L/v/tan(delta)/cos(beta)*(beta -atan((v*sin(beta)-x_f*v/L*tan(delta)*cos(beta))/(v*cos(beta)-y_f*v/L*tan(delta)*cos(beta))));

        %% Variable Steering Case - Modify Delta with feedback term
        Gamma = -atan(y_f/x_f);

        % Additional steer
        K_psi = 6.5;
        Psi = K_psi *Gamma;

        Psi = min(max(Psi, -RWA_max-delta), RWA_max-delta); % Added Psi not exceeding RWA_max limit

        decay_a = 0.1;   % Rate of decay
        decay_b = 500; % x_f distance from which the decay of gain starts
        decay_gain = (1+exp(-decay_a*(abs(x_f)-decay_b))).^-1;

        if Variable_Steer_Gain_Decay ==1
            delta = delta + Psi*Variable_Steer*decay_gain;
        else
            delta = delta + Psi*Variable_Steer;
        end
        %% Stanley Control

        if Stanley_Approach==1
            delta_stan = stanley_control(Coupler_estim_loc,Hitchball_world,smoothedPoint,trailer_angle,theta_animation,RWA_max,v);

            if norm(smoothedPoint) >2000 % only use Stanley at distances>2m
                delta = delta_stan;
            end
        end

        %%

        Delta_hist_deg(i,1) = delta*180/pi;
        delta_hist(i,1) = delta;

        if t==0
            initial_delta = delta;
        end

        w =2500;
        l = 5900;

        %     x_animation = -x_f_unrotated;
        %     y_animation = -y_f_unrotated;
        x_animation = -x_f_unrotated_noise_free;
        y_animation = -y_f_unrotated_noise_free;
        a=0;


        if ~isempty(theta_hist)
            theta_animation = theta_hist(end);
        end

        sim_time_per_run = min(T,Ts);
        if sim_time_per_run < Ts
            hitched = 1;
        end

        %     for t=0:Ts:sim_time_per_run
        % Run one time step
        [dx, dy, dv, dtheta] = bicycleModelDynamics_atHitch(theta_animation, v, delta, L, a,x_bar);

        x_animation = x_animation + dx*sim_time_per_run;
        y_animation = y_animation + dy*sim_time_per_run;
        v = v + dv*sim_time_per_run;
        theta_animation = theta_animation + dtheta*sim_time_per_run;

        x_rear = x_animation + x_bar*cos(theta_animation);
        y_rear = y_animation + x_bar*sin(theta_animation);


        x_hist = [x_hist; x_animation];
        y_hist = [y_hist; y_animation];
        dx_hist = [dx_hist; dx*sim_time_per_run];
        dy_hist = [dy_hist; dy*sim_time_per_run];
        theta_hist = [theta_hist; theta_animation];

        t=t+Ts;

        i=i+1;
    else
        hitched=hitched+1;
    end

    if Live_plot==1
        plot_update_sim;
    end
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
Coupler_estim_loc_hist_rel = Coupler_estim_loc_hist-Hitchball_world_hit(:,1:end-1);
% Coupler_estim_loc_hist_rel = [Coupler_estim_loc_hist_rel(:,1), Coupler_estim_loc_hist_rel]

figure(1010);
for j = 1:i-1
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
    if Use_Kalman
        scatter(Coupler_estim_loc(1), Coupler_estim_loc(2),99,'g+','LineWidth',1.8);
    end

    hold on; plot(x_hist(1:j),y_hist(1:j),'r-.');
    delete(findall(gcf,'type','annotation'));

    if Stanley_Approach==1
        Stanley_x_path = Coupler_estim_loc_hist(1,j) + [0:100:15000] * cos(trailer_angle);
        Stanley_y_path = Coupler_estim_loc_hist(2,j) + [0:100:15000] * sin(trailer_angle);
        plot(Stanley_x_path,Stanley_y_path, '-k','Color',[0,0,0,.2]);
    end
    % Plot future traj based on current steering
    [traj_x,traj_y] = get_future_traj(x_hist(j),y_hist(j),theta_hist(j), v, delta_hist(j), L, a,x_bar,Ts);
    plot(traj_x,traj_y,':k');

    if delta>RWA_max || delta <-RWA_max
        rwa_color = 'red';
    else
        rwa_color = 'black';
    end

    if Show_RWA_on_video==1
        A1 = annotation('textbox', [0.2, 0.73, 0.1, 0.1],  'String', {['rwa = ', num2str(floor(delta*180/pi*10)/10),' deg']}, ...
            'FontSize', 12,  'FontWeight', 'bold',  'Color', rwa_color, 'BackgroundColor', 'white',  'EdgeColor', 'black', ...
            'HorizontalAlignment', 'center',  'VerticalAlignment', 'middle');
        set(A1, 'horizontalAlignment', 'left');
        if Use_Kalman==1
            A2 = annotation('textbox', [0.2, 0.35, 0.1, 0.1],  'String', {['True Hitch2Coupler = [', num2str(round(x_hist(j))),',',num2str(round(y_hist(j))),'] mm'],...
                ['EstCoupler2TrueCoupler = [', num2str(round(Coupler_estim_loc(1))),',',num2str(round(Coupler_estim_loc(2))),'] mm']}, ...
                'FontSize', 12,  'FontWeight', 'bold',  'Color', 'k', 'BackgroundColor', 'white',  'EdgeColor', 'black', ...
                'HorizontalAlignment', 'center',  'VerticalAlignment', 'middle');
        else
            A2 = annotation('textbox', [0.2, 0.35, 0.1, 0.1],  'String', {['True Hitch2Coupler = [', num2str(round(x_hist(j))),',',num2str(round(y_hist(j))),'] mm'],...
                ['Coupler2TrueCoupler = [', num2str(round(Coupler_estim_loc(1))),',',num2str(round(Coupler_estim_loc(2))),'] mm']}, ...
                'FontSize', 12,  'FontWeight', 'bold',  'Color', 'k', 'BackgroundColor', 'white',  'EdgeColor', 'black', ...
                'HorizontalAlignment', 'center',  'VerticalAlignment', 'middle');
        end
        set(A2, 'horizontalAlignment', 'left');
    end

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