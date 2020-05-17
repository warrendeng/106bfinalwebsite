function [dxdt]=Circle_CBF_2WheelRobot_ODEfile(t,x)

%x(1)=x_1 (x position of robot)
%x(2)=x_2 (y position of robot)
%x(3)=x_3 (velocity of robot)
%x(4)=x_4 (angular position of robot)
%x(5)=x_5 (angular velocity of robot)


%% Reference Constants and Drag Force
global con;
%% Leader Feedback Controller
phi = atan2(x(2),x(1));

%% Control Lyapunov Function

A_clf = [2*(x(3) - con.follow.v_d)/con.mass 1];
b_clf = [ -(2*(x(3)-con.follow.v_d)/con.mass)*con.B + con.eps*(x(3)-con.follow.v_d)^2];

%% Control Barrier Function
h = 1/4*(con.R_1-con.R_2)^2 - (con.R_1+1/2*(con.R_2-con.R_1)-sqrt(x(1)^2+x(2)^2))^2;
h_dot = ((3-sqrt(x(1)^2+x(2)^2))*2*(x(1)*x(3)*cosd(x(4))+x(2)*x(3)*sind(x(4))))/(sqrt(x(1)^2+x(2)^2));
E = con.a_e*(con.b_e*h_dot^2)/(1+con.b_e*h_dot^2);
E_dot = (-2*con.a_e*con.b_e^2*h_dot^3)/(1+con.b_e*h_dot^2)^2 + (2*con.a_e*con.b_e*h_dot)/(1+con.b_e*h_dot^2);
B_2 = 1/h + E;
L_f_f = -(2*x(1)*x(3)*cosd(x(4))+2*x(2)*x(3)*sind(x(4)))^2/(2*(x(1)^2+x(2)^2)) - ((3-sqrt(x(1)^2+x(2)^2))*(2*x(1)*x(3)*cosd(x(4))+2*x(2)*x(3)*sind(x(4)))^2)/(2*(x(1)^2+x(2)^2)^(3/2))...
    + ((3-sqrt(x(1)^2+x(2)^2))*(2*x(3)^2*cosd(x(4))^2+2*x(3)^2*sind(x(4))^2-2*x(1)*x(3)*x(5)*sind(x(4))+2*x(2)*x(3)*x(5)*cosd(x(5))))/(sqrt(x(1)^2+x(2)^2));
L_g_f = ((3-sqrt(x(1)^2+x(2)^2))*2*(x(1)*cosd(x(4))+x(2)*sind(x(4))))/(sqrt(x(1)^2+x(2)^2));

A_cbf = [E_dot*L_g_f 0];
b_cbf = +1/h^2*h_dot + 1/B_2 - E_dot*L_f_f;

%% Calculate Force Control Input
A = [A_clf; A_cbf];
b = [b_clf; b_cbf];

H = 2*[1/con.mass^2 0; 0 con.p_sc]; %Cost Function H Matrix defined in Paper
f = [-2*con.B/con.mass^2; 0]; %Cost Function F Matrix defined in Paper


opts=optimset('quadprog');
            opts.Algorithm='interior-point-convex';
            opts.UseParallel='true';
            opts.Display='off';
            opts.Diagnostics='off';
            opts.TolX = 0.0001;
            opts.MaxFunEvals=1e6;
            opts.MaxIter=1000;
            X = quadprog(sparse(H),double(f),A,b,[],[],[],[],[],opts);
            %if ExitFlag < 1
                %fprintf('QP can not generate a legal force! \n')
                %keyboard
            %end
            
      
            u=X(1); %extract control input from quadprog solution
            
%% System of ODE's

%%%%%%%%%%%%% Leader Car %%%%%%%%%%%%%%%%%%%
%dx_1/dt
dxdt(1)= x(3)*cosd(x(4));

%dx_2/dt
dxdt(2)= x(3)*sind(x(4));
%dx_3/dt
dxdt(3) = u/con.mass;

%dx_4/dt
dxdt(4) = 0.5 * cos(0.5 * t); % I am not sure about this but if we use 1500 or so as coefficient the system shows stability

%dx_5/dt
dxdt(5) = 0; % we want that omega should be zero so is omega_dot
%input('hello:');
dxdt=dxdt'; % we need column vector.

end
