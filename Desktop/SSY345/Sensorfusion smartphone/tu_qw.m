function [x, P]=tu_qw(x, P, omega, T, Rw)
%F(w_k-1)
F=eye(4)+0.5*Somega(omega)*T;
%G(qk-1)
G=eye(4)+0.5*Sq(x)*T;

%EKF prediction step
x=F*x;
P=F*P*F'+G*Rw*G';

end