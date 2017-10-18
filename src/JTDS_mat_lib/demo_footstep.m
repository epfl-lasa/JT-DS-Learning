close all
clear
clc
[this_folder,~,~] = fileparts(mfilename);
addpath(genpath(strcat(this_folder,'../../')));

% This demo creates a 3-DOF robot leg, and a number of hand-designed
% trajectories with added random perturbations. It then trains the
% Joint-Space Task-Directed Dynamical System model on these demonstrations,
% and visualizes the resulting motions. Near the end, the file can generate
% a number of insightful figures on the behavior in latent-space and
% velocity-space of the learned controller.
dimq = 3;
% The following are the DH parameters for our 3 DOF robot
A = [.4, .4, .1];
D = [0, 0,0]; % All components are in the 2D plane
Alpha = [0, 0,0]; 
Qmin = -pi*[1, 1, -.25]; % No joint limits except for on the ankle
Qmax = pi*[1, 1, .75];
% Create a model of the robot
robot = initialize_robot(A,D,Alpha,Qmin,Qmax);
fig = initialize_robot_figure(robot);             
% Create a model plant for the robot's motor controller
robotplant = RobotPlant(robot, 'end_trans');

% The time between demonstration steps
dt = .005;

% We create two demonstrated motions, corresponding to the forward swing 
% and backward swing of a footstep.

% In the forward swing trajectory, the knee upward first, followed
% afterward by the foot (as occurs in human footsteps).
demo_basic{1} = [
    linspace(-pi/2 -.4, - pi/2 + .6, 50) linspace( - pi/2 + .6, -pi/2 + .5, 50);
    linspace(-.2, -pi/2, 50) linspace( -pi/2, -.5, 50);
    linspace(pi/2 - .3, pi/2, 50) linspace(pi/2, pi/2 + .05, 50)
    ];

% In teh backward swing trajectory, the whole foot moves backward in one
% large motion.
demo_basic{2} = [
    linspace(-pi/2 + .5, -pi/2 -.4, 100);
    linspace(-.5, -.2, 100);
    linspace(pi/2 + .05,pi/2 - .3, 100);
    ];

% Since we need sufficient training data, we'll generate random
% perturbations of these two base trajectories.
num_demos_generated = 10;
demos_perturb = pi/30*(rand(dimq, num_demos_generated)-.5);
j = 1;
for i = 1:num_demos_generated
    demos{i} = demo_basic{j} + repmat(demos_perturb(:, i), 1, size(demo_basic{1}, 2));
    j = mod(j, 2) + 1;
    
    % Uncomment the line below to see the training trajectories:
    
    %[fig, ~] = PlaybackTrajectory(robotplant, demos{i}, dt, fig);
end
%%

time = dt;
tol_cutting = .1; % Used in preprocessing to cut off the unnecessary ends of
% the trajectories where the velocity is below a specified threshold
% = tol_cutting
[Data, index] = preprocess_demos_jtds(robotplant, demos, time, tol_cutting);

% A set of options that will be passed to the JT-DS learner.
options.latent_mapping_type = 'Autoencoder';
options.autoencoder_num_dims = 2;
options.GMM_sigma_type = 'full'; % Can be either 'full' or 'diagonal'
options.explained_variance_threshold = .98; % How much of original data should be explained by latent-space projection
options.GMM_maximize_BIC = false; % If false, always use "options.fixed_num_gaussians" Gaussians in GMM model
options.fixed_num_gaussians = 3;
options.max_gaussians = 8; % Maximum number of Gaussians allowed in learned GMM
options.BIC_regularization = 2.5; % this should probably be between 1 and 3 - the higher it is, the fewer Gaussians will be used
options.verbose = true;
options.learn_with_bounds = true; % If false, does not incorporates joint limits in learning

% Finally, call the following function to learn the parameters of the
% model.
[Priors, Mu, Sigma, As, latent_mapping] = JTDS_Solver(Data,robotplant,options);
%% To simulate the learned trajectory
% Create a controller (MotionGeneratorBounded)
motion_generator_learned = MotionGeneratorBounded(robotplant, Mu, Sigma, Priors, As, latent_mapping);
% We also create a controller without the learned parameters
[~, identity_mapping] = compute_mapping(eye(dimq), 'None');
motion_generator_unlearned = MotionGeneratorBounded(robotplant, zeros(dimq, 1), eye(dimq), 1, eye(dimq), identity_mapping);
q_initial = [-pi/2-.5; -0.2; pi/2]; % Starting robot configuration
% goal task positions VVV
x_targets = [[.4; -.6; 0]];%, [.45; -.75; 0], [-.35; -.7; 0]];%, [.2; -.75; 0], [-.4; -.7; 0],[.35; -.6; 0], [.2; -.75; 0], [-.4; -.7; 0]];
max_trajectory_duration = 60; % How long to interpolate the trajectory
goal_tolerance = 0.05; % Distance from the goal to accept
% The following function interpolates the motion forward for either 60
% seconds or until we move within the goal's tolerance:
[Q_traj_learned, T_traj_learned] = computeFullTrajectory(q_initial, x_targets, motion_generator_learned, goal_tolerance, max_trajectory_duration);
[Q_traj_unlearned, T_traj_unlearned] = computeFullTrajectory(q_initial, x_targets, motion_generator_unlearned, goal_tolerance, max_trajectory_duration);
%% Compute the RMSE of velocity prediction
rmse_learned = mean(trajectory_error(motion_generator_learned, Data(1:3, :), Data(4:6, :), Data(7:9, :)));
rmse_unlearned = mean(trajectory_error(motion_generator_unlearned, Data(1:3, :), Data(4:6, :), Data(7:9, :)));
%% Visualize the learned motion
if ~ishandle(fig)
    fig = initialize_robot_figure(robot);
else
    figure(fig);
end
plot3(x_targets(1,:),x_targets(2,:),x_targets(3,:), 'ro', 'markersize', 20);
PlaybackTrajectory(robotplant, Q_traj_learned, T_traj_learned, fig);
% If you'd like to see the unlearned  motion, uncomment this line:
% PlaybackTrajectory(robotplant, Q_traj_unlearned', T_traj_unlearned', fig);

% The rest of the code allows you to create various graphs to better
% understand the robot's motion.

%% Plot 2D latent-space representation of robot's motion
figure(3)
for i = 1:num_demos_generated
    Z(:,:,i) = out_of_sample(demos{i}', latent_mapping)';
end
hold on
plotGMM(Mu(1:2, :), Sigma(1:2, 1:2, :), [.1, .3, .1], 1);
h6 = plot(Mu(1,1),Mu(2,1),'.');
for i = 1:num_demos_generated
    if mod(i, 2) == 0
        h3=plot(Z(1, 1:4:100,i), Z(2, 1:4:100,i), 'b.', 'markersize', 10);
    else
        h4=plot(Z(1, 1:4:100,i), Z(2, 1:4:100,i), 'r.', 'markersize', 10);
    end
end
%%
Q_new = Q_traj_learned; T_new = T_traj_learned;
Qd_new = [diff(Q_new, 1, 2)./repmat(diff(T_new), dimq, 1) zeros(dimq, 1)];
Z_new = out_of_sample(Q_new', latent_mapping)';
h1 = plot(Z_new(1, 1:5:end), Z_new(2, 1:5:end), 'k', 'linewidth', 3);%, 'r.', 'markersize', 20);

Q_new_default = Q_traj_unlearned; T_new_default = T_traj_unlearned;
Qd_new_default = [diff(Q_new_default, 1, 2)./repmat(diff(T_new_default), dimq, 1) zeros(dimq, 1)];
Z_new_default = out_of_sample(Q_new_default', latent_mapping)';
h2 = plot(Z_new_default(1, 1:5:end), Z_new_default(2, 1:5:end), 'm', 'linewidth', 3);%, 'r.', 'markersize', 20);

xlabel('\phi_1(q)'); ylabel('\phi_2(q)');


z_init = out_of_sample(q_initial', latent_mapping)';
h5=plot(z_init(1), z_init(2), 'color', 'red', 'markersize', 15, 'LineWidth', 3);

h = [h1 h2 h3 h4];
l = legend(h, 'Learned Trajectory', 'Classic Jacobian Transpose','Back-swing demonstration positions','Forward-swing demonstrationpositions');
set(l, 'FontSize', 15);
title('Latent-space representation of learned behavior');

% Create charts of relative joint velocity
figure(2)
hold on
for i = 1:dimq
    ax1 = subplot(3,1,1);
    hold on
    plot(dt*(1:100), Data(i + 3, 1:100), 'LineWidth', 2.5);
    ax2 = subplot(3,1,2);
    hold on
    plot(T_new_default, Qd_new_default(i, :), 'LineWidth', 2.5);
    ax3 = subplot(3,1,3);
    hold on
    plot(T_new, Qd_new(i, :), 'LineWidth', 2.5);
end
ax1.XLim = [0, 1];
linkaxes([ax1, ax2, ax3], 'x')
l = legend('Thigh (q_1)', 'Knee (q_2)', 'Ankle (q_3)');
set(l, 'FontSize', 14)
hold off;
% Create freezeframes of robots at different timesteps
GraphRobot(robot, Q_new(:, 1:ceil(end/4):end));
GraphRobot(robot, Q_new_default(:, 1:ceil(end/4):end));