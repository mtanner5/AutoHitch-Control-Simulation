function [tout, qout] = Simulator(t_span, q0, u_spline)
t_spline = linspace(t_span(1), t_span(2), numel(u_spline));
u = @(t) interp1(t_spline, u_spline, t);

odefun = @(t, q) dynamics(t, q, u(t));

[tout, qout] = ode45(odefun, t_span, q0);