clc; clear; close all;

N = 15;
p = getParams();
RWA_min = -30*pi/180;
RWA_max = 30*pi/180;

x0 = zeros(N+1, 1);
dist = sqrt(14000^2 + 3000^2);
x0(end) = dist/450;
cost = @(x) 0;

A = []; b = []; Aeq = []; beq = [];
lb = RWA_min*ones(N, 1);
ub = RWA_max*ones(N,1);

options = optimoptions("fmincon", "Display", "iter");

x_star = fmincon(cost, x0, A, b, Aeq, beq, lb, ub, @nonlcon, options);

t_star = linspace(0, x_star(end), N);
delta_star = x_star(1:end-1);

t_span = [0, x_star(end)];
q0 = [14e3; 3e3; 0];
[tout, qout] = Simulator(t_span, q0, delta_star);

figure()
plot(qout(:, 1), qout(:, 2))
xlabel('x')
ylabel('y')
title('Optimal Trajectory: Position')

figure()
plot(t_star, delta_star*180/pi)
xlabel('Time (s)')
ylabel('RWA (deg)')
title('Optimal Trajectory: RWA')