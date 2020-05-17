function dxu = compute_velocity(x,u,dt,m,Iz,a)
f = @(t,x) [x(3) * cos(x(4)) - a * x(5) * sin(x(4)); x(3) * sin(x(4)) + a * x(5) * cos(x(4)); -a * x(5)^2; x(5); 0];
g = @(t,x) [0 0;0 0; 1/m 0; 0 0; 0 1/Iz];
ul = [u(1); u(2)];
tspan = [0 dt];
init_x = x;
kinematics = @(t,x)(f(t,x) + g(t,x) * ul);
% p0 = kinematics(0,x);
% pt = kinematics(dt,x);
[t,x] = ode45(@(t,x) kinematics(t,x),tspan,init_x);
dxu = [mean(x(:,3));mean(x(:,5))];
end
