function [x, P]=tu_q(x, P, R)
% update
    x = x; %updated mean remain the same
    P = P + R; %updated covariance when gyr measurements are not avaliable
end