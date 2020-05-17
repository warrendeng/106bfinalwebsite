% Definition of constants in the 2D problem
function con = Constants_Globals()
	%con = {};
    mass = 1650;
    
	% Dynamics
	con.mass = 1;	% kg
    
    con.eps = 10;
    con.gamma = 1;
    con.c_a = 0.3;
    con.c_d = 0.3;
    con.g = 9.81;
    con.p_sc = 1;

	con.umin = -0.3*9.81*mass;	% Newton
	con.umax = 0.2*9.81*mass;	% Newton


	% Problem parameters
    con.B = 1; % damping coefficient (arbitrary)
    con.R = 1; % m radius of circle
    con.k = [1 1 10 10]; %position and velocity feedback gains
    con.R_1 = 4; %outer Radius of the circle
    con.R_2 = 2; % inner radius of the circle
    con.a_e = 10;
    con.b_e = 10;
    
    % Leader Car Properties
	con.leader.velocity = 3;
    con.leader.maxAccel = 20;
    
    % CutIn Car Properties
    con.cutin.maxAccel = 20;
    
    % Follower Car Properties
	con.follow.v_d = 1;
    con.follow.maxAccel = 25;
	con.follow.tau_des = 1.8;
    con.rate = -0.25;
    con.beta = 0.02;
    
    %Time Counter Variables
    con.i = 1;
    con.time1 = [];
    con.headDesCut1 = -0.7;
    con.j = 1;
    con.time2 = [];
    con.headDesCut2 = -0.46;


end