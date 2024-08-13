function [C, Ceq] = nonlcon(x)
p = getParams();

t_span = [0, x(end)];
q0 = [p.x_b; p.y_b; 0];
u_spline = x(1:end-1);

[tout, qout] = Simulator(t_span, q0, u_spline);

x_final_sim = qout(end, 1);
y_final_sim = qout(end, 2);
theta_final_sim = qout(end, 3);

x_final = p.x_c;
y_final = p.y_c;

% Inadmissible region
% circle: x^2 + y^2 <= c
% avoid circle: x^2 + y^2 >0 c
% c - x^2 - y^2 <= 0
c = 200;
inadmissible = [c - qout(:, 1).^2 - qout(:, 3).^2];

C = inadmissible;
Ceq = [x_final_sim - x_final; y_final_sim - y_final]