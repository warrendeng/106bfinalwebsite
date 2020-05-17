function [A,b_qp] = curate_QP(x,vd,omegad,P,b,R,a,n,m,Iz,dmax,D,tau,c,gamma,u0)
V1 = @(x)curate_V1(x,vd);
V2 = @(x)curate_V2(x);
V3 = @(x)curate_V3(x,R,b,n,m,Iz,a,P,u0);
    
hasr = @(x)curate_hsr(x,D,tau);
hlk = @(x)curate_hlk(x,dmax,n,R,b);
    
f = @(x) [x(3) * cos(x(4)) - a * x(5) * sin(x(4)); x(3) * sin(x(4)) + a * x(5) * sin(x(4)); -a * x(5)^2; x(5); 0];
g = @(x) [0 0;0 0; 1/m 0; 0 0; 0 1/Iz];

Lg = g(x);
Lf = f(x);
[J1,err1] = jacobianest(V1,x);
[J2,err2] = jacobianest(V2,x);
[J3,err3] = jacobianest(V3,x);
[Jasr,err4] = jacobianest(hasr,x);
[Jlk,err5] = jacobianest(hlk,x);
% test = -Jasr * Lg 
% test2 = gamma(1) * hasr(x)
func = [V1(x);V2(x);V3(x);hasr(x);hlk(x)];

A = [ J1 * Lg -1 0 0 ;J2 * Lg 0 -1 0 ; J3 * Lg 0 0 -1;Jasr * Lg 0 0 0;Jlk * Lg 0 0 0];
b_qp = [-J1 * Lf - c(1) * func(1); -J2 * Lf - c(2) * func(2); -J3 * Lf - c(3) * func(3);-Jasr * Lf - gamma(1) * func(4);-Jlk * Lf - gamma(2)*func(5)];
end
function V1 = curate_V1(x,vd)
% v1 = [x(3);vd];
V1 = (x(3) - vd)^2;
end
function V2 = curate_V2(x)
V2 = x(5)^2;
end

function V3 = curate_V3(x,R,b,n,m,Iz,a,P,u0)
f = @(x) [x(3) * cos(x(4)) - a * x(5) * sin(x(4)); x(3) * sin(x(4)) + a * x(5) * cos(x(4)); -a * x(5)^2; x(5); 0];
g = @(x) [0 0;0 0; 1/m 0; 0 0; 0 1/Iz];
phi = atan2(x(2),x(1));
phi_dot = (a * x(5) * cos(phi - x(4)) - x(3) * sin(phi - x(4)))/sqrt(x(1)^2 + x(2)^2);
Rpath = R+ b * sin(n * phi);
Rpath_dot = n * b * phi_dot * cos ( n * phi);
u = [u0(1); u0(2)];
x_dot = f(x) + g(x) * u;
eta = [x(1) - Rpath * cos(phi); x(2) - Rpath * sin(phi)];
eta_dot = [x_dot(1) - Rpath_dot * cos(phi) + Rpath * phi_dot * sin(phi);x_dot(2) - Rpath_dot * sin(phi) + Rpath * phi_dot * cos(phi)];
V3 = [eta' eta_dot'] * P * [eta' eta_dot']' ;
end
function hsr = curate_hsr(x,D,tau)
hsr = D - tau * x(3);
end
function hlk = curate_hlk(x,dmax,n,R,b)
phi = atan2(x(2),x(1));
Rpath = R + b * sin(n * phi);
Rpose = sqrt(x(1)^2 + x(2)^2);
ylat = (Rpath - Rpose)/dmax;
vlat = x(3) * cos(x(4) - phi);
hlk = (1- ylat^2 - 0.5 * vlat^2);
end

    
    
    
    