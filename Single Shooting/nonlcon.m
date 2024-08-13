function [C, Ceq] = nonlcon(x)
C = [];

t_span = [0, x(end)];
q0 = [14e3; 3e3; 0];
u_spline = x(1:end-1);

[tout, qout] = Simulator(t_span, q0, u_spline);

x_final_sim = qout(end, 1);
y_final_sim = qout(end, 2);
theta_final_sim = qout(end, 3);

x_final = 0;
y_final = 0;

Ceq = [x_final_sim - x_final; y_final_sim - y_final];