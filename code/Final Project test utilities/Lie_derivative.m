x = 1;
y = 1;
psi = pi/4;
v = 0.6;
omega = 0.0002;
xs = [x,y,v,psi,omega]';

v = @(x)(x(1)^2 + x(2)^2 + x(3) * cos(x(4))+ x(5)* cos(atan2(x(2),x(1))));
[A,err] = jacobianest(v,xs)

