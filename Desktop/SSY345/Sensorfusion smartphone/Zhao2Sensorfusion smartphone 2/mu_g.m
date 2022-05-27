function [x, P] = mu_g(x, P, yacc, Ra, g0)
% Input:
% yacc Measurement at k
% Ra measurement noise covariance
% g0 nominal gravity when flat on table
% Output:
% x  update mean
% P update covariance

[dQ0, dQ1, dQ2, dQ3] = dQqdq(x); %Derivative dQq/dq, using given computing function 'dQqdq.m'
hx = Qq(x)'*g0; %Measurement estimates, using given computing function 'Qq.m'. Force f_k^a is assumpted as 0
% Hx = [dQ0', dQ1', dQ2', dQ3'] .* g0; %Jacobian
Hx = [dQ0'* g0 dQ1'* g0 dQ2'* g0 dQ3'* g0]; %Jacobian

S = Hx * P * Hx' + Ra; %Innovation
K = P * Hx' / S; %Kalman filter gain

x = x + K * (yacc - hx); %Updated mean
P = P - K * S * K'; %Updated covariance

end