%% Problem 1 Part b
clear; clc; close all;
p = getParams;

%% Define key components of optimization
% Number of nodes
N = p.N;

% Decision vector
tau = sym('tau', [N 1]); % Input forces
s = sym('s', [N 1]); % Displacement
ds = sym('ds', [N 1]); % Velocity

w = [tau; s; ds];

% Cost function
syms sfinal dsfinal
Ts = p.Ts;
cost = Ts*sum(([s-sfinal; ds-dsfinal]).^2);

%% Constraints
% Defect constraints
h = Ts;
I = p.I;
b = p.b;

tau_k = tau(1:end-1);
s_k = s(1:end-1);
s_k1 = s(2:end);
ds_k = ds(1:end-1);
ds_k1 = ds(2:end);
dds_k = (-b*ds_k + tau_k)/I;

defect_pos = s_k1 - s_k - ds_k*h; % s_k+1 = s_k + ds_k*h
defect_vel = ds_k1 - ds_k - dds_k*h; % ds_k+1 = ds_k + dds_k*h
defect = [defect_pos; defect_vel];

% Initial conditions
syms s0 ds0
init_cond = [s(1) - s0; ds(1) - ds0]; % s(1) = s0; ds(1) = 0

% Inequality saturation limits
% Aw <= b
tau_max = p.tau_max;
tau_min = p.tau_min;

limits = [tau - tau_max; -tau + tau_min];

% Equality constraints
% Ceq(w) = 0
Ceq = [defect; init_cond];

% Inequality constraints
Cineq = limits; % Limits are only ineq constraints

%% Hessians, gradients, constraints, etc
H = hessian(cost, w);
c = gradient(cost, w);
c = subs(c, w, zeros(size(w)));

Aeq = jacobian(Ceq, w);
beq = -subs(Ceq, w, zeros(size(w)));

A = jacobian(Cineq, w);
b = -subs(Cineq, w, zeros(size(w)));

x0 = [s0; ds0];
xf = [sfinal; dsfinal];

%% Generate functions
matlabFunction(H, 'File', 'Hfunc', 'Vars', {x0, xf})
matlabFunction(c, 'File', 'cfunc', 'Vars', {x0, xf})
matlabFunction(Aeq, 'File', 'Aeqfunc', 'Vars', {x0, xf})
matlabFunction(beq, 'File', 'beqfunc', 'Vars', {x0, xf})
matlabFunction(A, 'File', 'Afunc', 'Vars', {x0, xf})
matlabFunction(b, 'File', 'bfunc', 'Vars', {x0, xf})