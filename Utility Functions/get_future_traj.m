function [traj_x,traj_y] = get_future_traj(x_animation,y_animation,theta_animation, v, delta, L, a,x_bar,Ts)

% sim for 7 sec
sim_T = 7;
traj_x= zeros(1,sim_T/Ts);
traj_y= zeros(1,sim_T/Ts);

i = 1;
for t=Ts:Ts:sim_T
[dx, dy, ~, dtheta] = bicycleModelDynamics_atHitch(theta_animation, v, delta, L, a,x_bar);

    x_animation = x_animation + dx*Ts;
    y_animation = y_animation + dy*Ts;
    theta_animation = theta_animation + dtheta*Ts;

    traj_x(i) = x_animation;
    traj_y(i) = y_animation;
    i=i+1;
end