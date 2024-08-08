% Kalman filter function
function [x_k_new, P_k_new] = kalman_filter_step(x_k, P_k, A, Q, measurement, R,Bu)
n = size(P_k,1);
% Prediction step
x_k_new = A * x_k + Bu; % Predict next state
P_k_new = A * P_k * A' + Q; % Predict next covariance

% Update step
H = eye(3, n); % Measurement matrix maps state to measurements
y_k = measurement - H * x_k_new; % Measurement residual
S = H * P_k_new * H' + R; % Residual covariance
K = P_k_new * H' / S; % Kalman gain

% Update state and covariance matrix
x_k_new = x_k_new + K * y_k;
P_k_new = (eye(n) - K * H) * P_k_new;
end