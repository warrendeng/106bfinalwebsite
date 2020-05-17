%% Get Robotarium object used to communicate with the robots/simulator
N = 2;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 5000;
a = 0.02;
vd = [0.05 0.075];
omegad = [0 0];
x_shift = zeros(5,N);
dt = 0.01;
s = sqrt(3);
P = [s 0 1 0;0 s 0 1;1 0 s 0;0 1 0 s];
p = [1 1 1e5 1 1e3];
H = diag(p);
m = 0.060; %mass of the robot
Iz = 0.0758; %0.00015; % Momemnt of inertia about z axis
R = 3*0.25; %Radius of the path in which robots move
n = 3; %Number of loops in the path
dmax = 0.04; %maximum allowable lateral displacement
b = 3*0.05; %amplitude of sinusoidal trajectroy
tau = 3; % seconds
%D = 0.03; %meters
c = [1; .1; 1.9]; %tunable parameter convergence rates
gamma = [1; 10]; %tunable parameter
u0 = [1 1 0 0 0]'; %initial force and torque inputs
%leader state to toggle between position controller and CLF-CBF-QP controller
state_leader = 1;
%follower state
state_follower = 1;

phi_desired = pi/6;
Rpath_desired = R + b * sin(n * phi_desired);
waypoint_desired_leader = [0; R];
waypoint_desired_follower = [R; 0];
R_inner = R -  3* dmax;
R_outer = R + 3 * dmax;

waypoints = [R 0];
waypoints_inner = [R_inner 0];
waypoints_outer = [R_outer 0];
phi1 = linspace (0,2*pi,100);
for phi = phi1
    flex = b * sin(n * phi);
    Rpath = R + flex;
    Rpath_inner = R_inner + flex;
    Rpath_outer = R_outer + flex;
    waypoint = [Rpath * cos(phi) Rpath * sin(phi)];
    waypoint_inner = [Rpath_inner * cos(phi) Rpath_inner * sin(phi)];
    waypoint_outer = [Rpath_outer * cos(phi) Rpath_outer * sin(phi)];
    waypoints = [waypoint;waypoints];
    waypoints_inner = [waypoint_inner;waypoints_inner];
    waypoints_outer = [waypoint_outer;waypoints_outer];
end
waypoints = waypoints';
waypoints_inner = waypoints_inner';
waypoints_outer = waypoints_outer';
wp = plot(waypoints(1,:),waypoints(2,:),'r.');
wp_inner = plot(waypoints_inner(1,:),waypoints_inner(2,:), 'b.');
wp_outer = plot(waypoints_outer(1,:),waypoints_outer(2,:),'g.');
wp_leader_place = plot(waypoint_desired_leader(1),waypoint_desired_leader(2),'bo');
wp_follower_place = plot(waypoint_desired_follower(1),waypoint_desired_follower(2),'go');
close_enough = .03;
opts = optimoptions(@quadprog,'Display','iter');
unicycle_barrier_certificate = create_uni_barrier_certificate_with_boundary();
initial_pose_leader = [0;R;pi];
initial_pose_follower = [R;0;3*pi/2];

args = {'PositionError', 0.03, 'RotationError', 0.05};
init_checker = create_is_initialized(args{:});
controller = create_waypoint_controller(args{:});
       


% Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
   %% Drive the leader car to its way point
   switch state_leader        
        case 1
            while(~init_checker(x(:,1), initial_pose_leader))
                dxu(:,1) = controller(x(:,1), initial_pose_leader);
                dxu(:,1) = unicycle_barrier_certificate(dxu(:,1), x(:,1));  
                r.set_velocities(1, dxu(:,1));
                r.step(); 
                x = r.get_poses();
            u = u0;
            state_leader = 2;
            end
        case 2
            if t == 1;% set initial conditions
                xs(:,1) = reconstruct_states(x(:,1),[vd(1);omegad(1)],a);
                [A,b_qp] = curate_QP_leader(xs(:,1),vd(1),omegad(1),P,b,R,a,n,m,Iz,dmax,c,gamma,u0);
                u = quadprog(sparse(H),[],A,b_qp,[],[],[],[],[],opts);
                dxu(:,1) = compute_velocity(xs(:,1),u,dt,m,Iz,a);
            
            else
                disp("I am leader");
                disp(u);
                xs(:,1) = reconstruct_states(x(:,1),[dxu(1,1);dxu(2,1)],a);
                [A,b_qp] = curate_QP_leader(xs(:,1),vd(1),omegad(1),P,b,R,a,n,m,Iz,dmax,c,gamma,u);
                u = quadprog(sparse(H),[],A,b_qp,[],[],[],[],[],opts);
                dxu(:,1) = compute_velocity(xs(:,1),u,dt,m,Iz,a);
            end
   end
%    % Drive the follower car to it designated pose
%    switch state_follower        
%         case 1
%             while(~init_checker(x(:,2), initial_pose_follower))
%                 
%                 dxu(:,2) = controller(x(:,2), initial_pose_follower);
%                 dxu(:,2) = unicycle_barrier_certificate(dxu(:,2), x(:,2));  
%                 r.set_velocities(2, dxu(:,2));
%                 r.step(); 
%                 x = r.get_poses();
%             u = u0;
%             state_follower = 2;
%             end
%         case 2
%             disp("Hello I am follower");
%             if t == 1;% set initial conditions
%                 xs(:,2) = reconstruct_states(x(:,2),[vd(2);omegad(2)],a);
%                 D = norm(x(1:2,1)-x(1:2,2));
%                 [A,b_qp] = curate_QP(xs(:,2),vd(2),omegad(2),P,b,R,a,n,m,Iz,dmax,D,tau,c,gamma,u0);
%                 u = quadprog(sparse(H),[],A,b_qp,[],[],[],[],[],opts);
%                 dxu(:,2) = compute_velocity(xs(:,2),u,dt,m,Iz,a);
%             
%             else
%                 xs(:,2) = reconstruct_states(x(:,2),[dxu(1,2);dxu(2,2)],a);
%                 D = norm(x(1:2,1)-x(1:2,2));
%                 [A,b_qp] = curate_QP(xs(:,2),vd(2),omegad(2),P,b,R,a,n,m,Iz,dmax,D,tau,c,gamma,u);
%                 u = quadprog(sparse(H),[],A,b_qp,[],[],[],[],[],opts)
%                 dxu(:,2) = compute_velocity(xs(:,2),u,dt,m,Iz,a);
%             end
%    end
    
    %%  Avoid Actuator errors
    % To avoid errors, we need to threshold dx
    norms = arrayfun(@(x) norm(dxu(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
%     to_thresh(1) = 0
    dxu(:, to_thresh) = threshold*dxu(:, to_thresh)./norms(to_thresh);
%     dxu = si_to_uni_dyn(dxu,x);
    r.set_velocities(1:N,dxu);
    r.step();
    pause(dt);
end

% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();


%% Helper functions
