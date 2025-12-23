%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();

disp('iTj_0')
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
q = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';

%% Define the tool frame rigidly attached to the end-effector
% Tool frame definition
gamma_t = [pi/10, 0 , pi/6];
eRt = YPRtoRot(gamma_t(1),gamma_t(2),gamma_t(3)); 
e_r_te = [0.3, 0.1, 0]';

eTt = [eRt, e_r_te;
         0,0, 0, 1];

disp("eTt");
disp(eTt);


%% Initialize Geometric Model (GM) and Kinematic Model (KM)

% Initialize geometric model with q0
gm = geometricModel(iTj_0,jointType,eTt);

% Update direct geoemtry given q0
gm.updateDirectGeometry(q);

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

bTt = gm.getToolTransformWrtBase();

disp("eTt");
disp(eTt);
disp('bTt q = 0');
disp(bTt);

%% Define the goal frame and initialize cartesian control
% Goal definition 
bOg = [0.2, -0.7, 0.3]';
gamma_g = [0, 1.57, 0];
bRg = YPRtoRot(gamma_g(1),gamma_g(2), gamma_g(3));

bTg = [bRg, bOg;
    0 ,0, 0, 1];

disp('bTg')
disp(bTg)

% control proportional gain 
k_a = 0.8;
k_l = 0.8;

% Cartesian control initialization
cc = cartesianControl(gm, k_a, k_l);






%% Initialize control loop 

% Simulation variables
samples = 100;
t_start = 0.0;
t_end = 20.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

% preallocation variables
bTi = zeros(4, 4, gm.jointNumber);
bri = zeros(3, gm.jointNumber+1);

% joints upper and lower bounds 
qmin = -3.14 * ones(7,1);
qmin(6) = 0;
qmax = +3.14 * ones(7,1);
qmax(6) = 1;

%%
show_simulation = true;
pm = plotManipulators(show_simulation);
pm.initMotionPlot(t, bTg(1:3,4));

counter = 0;


%%%%%%% Kinematic Simulation %%%%%%%
for i = t
    counter = counter +1;
    % Updating transformation matrices for the new configuration 
    gm.updateDirectGeometry(q);
    % Get the cartesian error given an input goal frame
    x_dot = cc.getCartesianReference(bTg); %non aggiorno bTg perche il goal è fermo
    % Update the jacobian matrix of the given model
    km.updateJacobian();

    %% INVERSE KINEMATICS
    % Compute desired joint velocities 
    q_dot =  pinv(km.J)* x_dot;

    % simulating the robot
    q = KinematicSimulation(q,q_dot, dt, qmin, qmax );
    
    pm.plotIter(gm, km, i, q_dot);
    
    disp("x_dot");
    disp(x_dot);
    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    end

end

pm.plotFinalConfig(gm);

%%
%Testing
gm = geometricModel(iTj_0,jointType,eTt);
gm.updateDirectGeometry(q); %final configuration
T = gm.getToolTransformWrtBase();

ep = bTg(1:3,4) - T(1:3,4);
Rerr = bTg(1:3,1:3) * T(1:3,1:3)';
[h, theta] = RotToAngleAxis(Rerr);
eo = h * theta;

disp('Position error norm:')
disp(norm(ep))

disp('Orientation error norm:')
disp(norm(eo))
disp("Final T");
disp(T);

bTg = [bRg, bOg;
    0 ,0, 0, 1];
disp("Desired T");
disp(bTg);

%% Q2.5


V_tb = km.J*q_dot;

bTt = gm.getToolTransformWrtBase();

bTe = gm.getTransformWrtBase(length(q));

bJe = km.getJacobianOfLinkWrtBase(length(q));

V_eb = bJe*q_dot;

o_w_tb = V_tb(1:3);
o_v_tb = V_tb(4:6);

o_w_eb = V_eb(1:3);
o_v_eb = V_eb(4:6);

disp('End-effector angular velocity :');
disp(o_w_tb);
disp('End-effector linear velocity:');
disp(o_v_tb);

disp('Tool angular velocity :');
disp(o_w_eb);
disp('Tool linear velocity:');
disp(o_v_eb);


