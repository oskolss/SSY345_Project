function [x, P] = mu_m(x, P, mag, m0, Rm)
% Input:
% mag Measurement at k from magnetometer
% Rm measurement noise covariance from magnetometer
% m0 itself
% Output:
% x update mean
% P update covariance

[dQ0, dQ1, dQ2, dQ3] = dQqdq(x); %Derivative dQq/dq, using given computing function 'dQqdq.m'
hx = Qq(x)' * m0; %Measurement estimates, using given computing function 'Qq.m'. Force f_k^m is assumpted as 0
Hx = [dQ0'*m0 dQ1'*m0 dQ2'*m0 dQ3'*m0]; %Jacobian

S = Hx * P * Hx' + Rm; %Innovation
%S = (S + S')/2;
K = P * Hx' / S; %Kalman filter gain

x = x + K * (mag - hx); %Updated mean
P = P - K * S * K'; %Updated covariance

end