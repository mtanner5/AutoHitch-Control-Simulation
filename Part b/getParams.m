%% Problem 1 Part b
function params = getParams()

params.N = 21;
params.T = 1;
params.Ts = params.T/(params.N - 1);
params.Tf = 5;
params.I = 2;
params.b = 0.5;
params.tau_min = -5;
params.tau_max = 5;