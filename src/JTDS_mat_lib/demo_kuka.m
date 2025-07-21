%% KUKA training demo
% This file implements a JT-DS training script for the KUKA LWR 4+.
% To get some pre-recorded KUKA demonstration data, check out
% github.com/epfl-lasa/rtk_JT_DS . In particular, you can use a
% demonstration like:
%
% https://github.com/epfl-lasa/rtk_JT_DS/tree/master/data/mat/fore_hand/raw_data.mat
%
% Whatever you use, replace the line below importing the demonstrations
% with your local demonstration data file.

close all
clear
clc
% 
% DH parameters for the KUKA LWR 4+ robot
dimq = 7;
A = [0 0 0 0 0 0 0.05];
Alpha = pi/2*[1 -1 -1 1 1 -1 0];
D = [.34 0 .4 0 .4 0 .279];
Qmin = 2*pi/180*[-85, -90, -100, -110, -140, -90, -120];
Qmax = 2*pi/180*[85, 90, 100, 110, 140, 90, 120];
% Create a model of the robot
robot = initialize_robot(A,D,Alpha,Qmin,Qmax);
fig = initialize_robot_figure(robot);             
% Create a model plant for the robot's motor controller
robotplant = RobotPlant(robot, 'end_trans');
%% Upload the demonstration data
% --------------- REPLACE THIS WITH KUKA DEMONSTRATIONS LOCATION ---------
% %demos_location = 'path/to/kuka/demo/raw_data.mat';
% demos_location = '~/Downloads/pour_no_obst/data.mat';
% % ------------------------------------------------------------------------
% [Qs, Ts] = ImportDemonstrations(demos_location);
% 
% % If the data is very dense, initializing the semidefinite program may take
% % a long time. In this case, it may help to thin down the number of
% % demonstrated points (by varying "thinning_ratio", so long as there are still sufficient points to
% % satisfactorily reconstruct the shape of the trajectory.
% thinning_ratio = 50; % In the KUKA case, we get 500 datapoints per second, so we shrink the data density considerably
% for i = 1:length(Qs)
%     Qs{i} = Qs{i}(:, 1:thinning_ratio:end);
%     Ts{i} = Ts{i}(:, 1:thinning_ratio:end);
% end


% -->> Should copy code for demo_learn_JTDS_kuka (first block) to load
% trajectory data
%%  Execute the learning phase

time = Ts;
demos = Qs;
tol_cutting = .1;
[Data, index] = preprocess_demos_jtds(robotplant, demos, time, tol_cutting);

% A set of options that will be passed to the SEDS solver. Please type 
% 'doc JTDS_Solver' in the MATLAB command window to get detailed
% information about other possible options.

options.latent_mapping_type = 'KPCA';
options.autoencoder_num_dims = 5;
options.GMM_sigma_type = 'full'; % Can be either 'full' or 'diagonal'
options.explained_variance_threshold = .9;
options.GMM_maximize_BIC = false;
options.fixed_num_gaussians = 5;
options.max_gaussians = 8;
options.BIC_regularization = 2; % this quantity should probably be between 1 and 3
options.verbose = true;

[Priors, Mu, Sigma, As, latent_mapping] = JTDS_Solver(Data,robotplant,options);
%% Export the learned values to a file
if strcmp(latent_mapping.name, 'PCA')
    M = latent_mapping.M;
else
    error('Need to devise a scheme to compress latent mappings - currently only PCA works')
end
export2JSEDS_Cpp_lib('~/Research/joint-pbd/kuka_demos/results/backhand_params.txt',Priors, Mu, Sigma, robot, As, M);

%% Visualize the results of the learning process in MATLAB
% First, create a controller which uses the learned policy
motion_generator_learned = MotionGeneratorBounded(robotplant, Mu, Sigma, Priors, As, latent_mapping);
[~, identity_mapping] = compute_mapping(eye(dimq), 'None');
motion_generator_unlearned = MotionGeneratorBounded(robotplant, zeros(dimq, 1), eye(dimq), 1, eye(dimq), identity_mapping);

%%
dt = 0.1; % the timestep
q_initial = [-0.728371429443359;-1.3605418920517;2.69252680319093;0.620675325393677;0.955035626888275;0.141930669546127;-0.17979271709919];
x_targets = [[-.5; -.5; 0.3]];
max_trajectory_duration = 60; % How long to interpolate the trajectory
goal_tolerance = 0.05; % How far away from the goal we need to get to accept
[Q_traj_learned, T_traj_learned] = computeFullTrajectory(q_initial, x_targets, motion_generator_learned, goal_tolerance, max_trajectory_duration);
[Q_traj_unlearned, T_traj_unlearned] = computeFullTrajectory(q_initial, x_targets, motion_generator_unlearned, goal_tolerance, max_trajectory_duration);

% Finally, display the learned motion
if ~ishandle(fig)
    fig = initialize_robot_figure(robot);
else
    figure(fig);
end
plot3(x_targets(1,:),x_targets(2,:),x_targets(3,:), 'ro', 'markersize', 20);
PlaybackTrajectory(robotplant, Q_traj_learned, T_traj_learned, fig);

% To compare the learned motion with the unlearned motion, replace
% Q_traj_learned with Q_traj_unlearned
%% Compute the RMSE of velocity prediction
rmse_learned = mean(trajectory_error(motion_generator_learned, Data(1:dimq, :), Data(dimq+1:2*dimq, :), Data(2*dimq+1:end, :)))
rmse_unlearned = mean(trajectory_error(motion_generator_unlearned, Data(1:dimq, :), Data(dimq+1:2*dimq, :), Data(2*dimq+1:end, :)))