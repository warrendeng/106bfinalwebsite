clear all
clc 
close all
warning off

%% Initialize Global Constants
con = Constants_and_Globals;
global con

%% Initial Conditions

x_1_int = 0.00; %Initial x position 
x_2_int = 0.000; %Initial y position
x_3_int = 0.002; % Initial velocity
x_4_int = 0.00; %Initial angular position
x_5_int = 0; %Initial angular velocity



% Time Integration Range
t_int = 0;
t_fin = 50;
dt = 0.01; %integration step size

%% Create x and t vectors

x_int = [x_1_int x_2_int x_3_int x_4_int x_5_int]; %Vector of Initial Values
t_span = [t_int:dt:t_fin]; %Time Integration Range Vector


%% Solving the ODE

%Specify relative error
options = odeset('RelTol', 1e-2, 'OutputFcn', @odephas2, 'OutputSel',[1,2],'NormControl','on')
[t,x]=ode45(@Circle_CBF_2WheelRobot_ODEfile,t_span,x_int,options)

%input('I am here:')


%Extract Solution
x_l = x(:,1); % robot x position
y_l = x(:,2); % robot y position
v_l = x(:,3); % robot velocity
theta = x(:,4); % robot angular position
omega = x(:,5); % robot angular velocity


%
figure(1)
axis square




%% Plotting
%Circle data Creating Code

th = 0:pi/50:2*pi;
for ii = 1:length(th)
    x_c(ii) = cos(th(ii));
    y_c(ii) = sin(th(ii));
end

%Plots position
figure
plot(x_l,y_l,'-b','linewidth',2)
hold on
plot(2*x_c,2*y_c,'-r','linewidth',2)
plot(4*x_c,4*y_c,'-r','linewidth',2)
plot(3*x_c,3*y_c,'--k','linewidth',2)
xlabel('X Position (m)')
ylabel('Y Position (m)')
title('2D Position of Leader')
axis([-5 5 -5 5],'square')
legend('Robot Path','Inner Path Limit','Outer Path Limit','Nominal Path')
grid on


%Plots speed of car
figure
plot(t,v_l)
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Velocity')
grid on