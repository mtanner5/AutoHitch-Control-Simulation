%% Problem 1 Part b
clear; clc; close all;
p = getParams;

% Simulation
I = p.I;
b = p.b;

% MPC
Ts = p.Ts;
Tf = p.Tf;

x0 = [0; 0];
xf = [3*pi/4; 0];

t_store = [];
x_store = [];
u_store = [];
for t_current = 0:Ts:(Tf-Ts)
    % Run trajectory optimization
    wstar = quadprog(Hfunc(x0, xf), cfunc(x0, xf), Afunc(x0, xf), bfunc(x0, xf), Aeqfunc(x0, xf), beqfunc(x0, xf));
    tau = wstar(1);

    dyn2 = @(t, x, tau) [x(2); (-b*x(2) + tau)/I];
    dynamics = @(t, x) dyn2(t, x, tau);

    % Run simulation for Ts seconds with optimized tau
    [tout, xout] = ode45(dynamics, [0 Ts], x0);

    % Store time, state, and force
    t_store = [t_store; tout + t_current];
    x_store = [x_store; xout];
    u_store = [u_store; ones(size(tout))*tau];

    % Resample state
    x0 = xout(end, :).';
end

% Plot position
figure()
subplot(3, 1, 1)
plot(t_store, x_store(:, 1))
xlabel('Time (s)')
ylabel('Position (rad)')

% Plot velocity
subplot(3, 1, 2)
plot(t_store, x_store(:, 2))
xlabel('Time (s)')
ylabel('Velocity (rad/s)')

% Plot torque
subplot(3, 1, 3)
plot(t_store, u_store(:, 1))
xlabel('Time (s)')
ylabel('Torque (Nm)')
sgtitle('Problem 1b: MPC Double Integrator with Minimum Error Solution')