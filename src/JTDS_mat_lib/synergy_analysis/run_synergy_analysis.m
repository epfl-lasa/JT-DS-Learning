%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Evaluation of Learning Schemes for JTDS Models (including orientation) on Different Datasets     %%
%  Train and compare a series of JTDS models with different dimensionality                          %%   
%  reduction schemes and GMM model fitting                                                          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;
do_plots  = 1;
data_path = '../../../Data/mat/'; % <-Insert path to datasets folder here
choosen_dataset = 'pour_obst_2'; % Options: 'back','fore','pour','pour_obst','foot','singularity';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load and Process dataset %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

switch choosen_dataset
    case 'back'
        demos_location = strcat(data_path, 'back_hand/data.mat');
        demo_ids = [2:11];
    case 'fore'
        demos_location = strcat(data_path,'fore_hand/data.mat');
        demo_ids = [1:11];
    case 'pour' 
        demos_location = strcat(data_path,'pour_no_obst/data.mat');
        demo_ids = [1 2 3 5 6 7 8 9 10];
    case 'pour_obst'
        demos_location = strcat(data_path,'pour_obst/data.mat');
        demo_ids = [1:10];
    case 'pour_obst_2'
        demos_location = strcat(data_path,'pour_obst_2/data.mat');
        demo_ids = [1:7];                
    case 'foot'        % This dataset was recorded at 50 Hz! thinning_ratio = 1 or 2
        demos_location = strcat(data_path,'foot/data.mat');
        demo_ids = [1:8];                
    case 'singularity'   
        demos_location = strcat(data_path,'singularity/data.mat');
        demo_ids = [1:10];  
        fprintf('Loading demonstrations from %s \n', demos_location);
        load(demos_location)
end

if ~strcmp(choosen_dataset,'singularity')
    fprintf('Loading demonstrations from %s \n', demos_location);
    [Qs_, Ts_] = ImportDemonstrations(demos_location);
end

% If the data is very dense, initializing the semidefinite program may take
% a long time. In this case, it may help to thin down the number of
% demonstrated points (by varying "thinning_ratio", so long as there are still sufficient points to
% satisfactorily reconstruct the shape of the trajectory.
% In the KUKA case, we get 500 datapoints per second, so we recommend shrinking the data density considerably
thinning_ratio = 20; % Same as demonstrations recorded at 10->50Hz, 20->25Hz
Qs = []; Ts= [];
for i = 1:length(demo_ids)
    Qs{i,1} = Qs_{demo_ids(i)}(:, 1:thinning_ratio:end);
    Ts{i,1} = Ts_{demo_ids(i)}(:, 1:thinning_ratio:end);
end

% Plot Full Set of Demonstrations per DOF
if do_plots
    figure('Color',[1 1 1])    
    Data_ = [];
    qdim = size(Qs{1},1);
    for dof=1:7
        subplot(qdim,1,dof)
        for i=1:length(Qs)
            data = Qs{i};
            data_dof = data(dof,:);
            plot(data_dof,'-.'); hold on;
        end
        grid on;
        title(sprintf('Raw Demonstrations for $q_%d$',dof), 'Interpreter', 'LaTex', 'Fontsize', 15)
        xlabel('Time (samples)','Interpreter', 'LaTex', 'Fontsize', 15)
        ylabel('Angle (rad)','Interpreter', 'LaTex', 'Fontsize', 15)
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Train a JTDS model on the current dataset   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Choose Lower-Dimensional Mapping Technique
mapping = {'PCA'}; % 'None', 'PCA', 'KPCA'

%%% Learning options %%%
options = [];
options.orientation_flag = 1;
options.tol_cutting = 0.1;

%%% Dim-Red options %%%
options.explained_variance_threshold = .95;
% If choosen mapping is K-PCA you need to choose the kernel width
% options.kpca_sigma = mean_D/sqrt(2);

%%% GMM options %%%
options.GMM_sigma_type = 'full'; % Can be either 'full' or 'diagonal'
options.GMM_maximize_BIC = true;
options.max_gaussians = 10;
options.plot_BIC = 0; 

% Optimization options 
options.learn_with_bounds = false;
options.verbose = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% DH parameters for the KUKA LWR 4+ robot %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dimq = 7;
A = [0 0 0 0 0 0 0.05];
Alpha = pi/2*[1 -1 -1 1 1 -1 0];
D = [.34 0 .4 0 .4 0 .279];
Qmin = 2*pi/180*[-85, -90, -100, -110, -140, -90, -120];
Qmax = 2*pi/180*[85, 90, 100, 110, 140, 90, 120];
% Create a model of the robot
robot = initialize_robot(A,D,Alpha,Qmin,Qmax);    
% Create a model plant for the robot's motor controller
robotplant = RobotPlant(robot, 'end_trans');

%%% Pre-process Data %%%
[Data_train, index] = preprocess_demos_jtds(robotplant, Qs, Ts, options.tol_cutting,options.orientation_flag);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Learn JTDS variants for current fold      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mapping_name = mapping{1};
fprintf('Training JTDS generator using %s mapping...\n', mapping_name);
options.latent_mapping_type = mapping_name;

% Run JTDS Solver function
[Priors, Mu, Sigma, As, latent_mapping] = JTDS_Solver_v2(Data_train, robotplant, options);
K = length(Priors);

if strcmp('PCA',latent_mapping.name)
    pca_dim = length(latent_mapping.lambda);
end

% Generate Trajectories from Learnt JTDS
motion_generator = MotionGenerator(robotplant, Mu, Sigma, Priors, As, latent_mapping);

% Compute RMSE on training data
rmse_train = mean(trajectory_error(motion_generator, Data_train(1:dimq, :), Data_train(dimq+1:2*dimq, :), Data_train(2*dimq+1:end, :),options.orientation_flag));
fprintf('Using %s mapping, got prediction RMSE on training: %d \n', mapping_name, rmse_train);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Plot Lower-Dimensional Embedding and Synergies   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extract Lower Dimensional Embedding of Demonstrations
figure('Color',[1 1 1])  
for p=1:pca_dim
    subplot(pca_dim,1,p)    
    for i=1:length(Qs)
        q_ref = Qs{i};
        phi_q = out_of_sample(q_ref', latent_mapping)';
        plot(phi_q(p,:),'-.'); hold on;
    end      
    grid on;
    title(sprintf('Raw Trajectories in PCA-space $\\phi_%d(q)$',p), 'Interpreter', 'LaTex', 'Fontsize', 15)
    xlabel('Time (samples)','Interpreter', 'LaTex', 'Fontsize', 15)   
end
% Plot in 3D-space
if pca_dim == 3
    
    K_colors = hsv(K);
    figure('Color',[1 1 1])
    for i=1:length(Qs)
        q_ref = Qs{i};
        phi_q = out_of_sample(q_ref', latent_mapping)';
        % Hard clustering for each local model
        labels =  my_gmm_cluster(phi_q, Priors, Mu, Sigma, 'hard', []);
        for k=1:K
            phi_q_k   = phi_q(:,labels==k);
            scatter3(phi_q_k(1,:),phi_q_k(2,:),phi_q_k(3,:),20,K_colors(k,:),'*'); hold on;
        end
        scatter3(phi_q(1,end),phi_q(2,end),phi_q(3,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
        axis tight
    end
    
    % Plot Gaussians for local behavior cluster --> might change this to
    % coloring the data-point with the posterior probability
    handles = my_plot3dGaussian(Priors, Mu, Sigma );
    grid on;
    title('Clustered (GMM) Trajectories in PCA-space $\phi(q)$', 'Interpreter', 'LaTex', 'Fontsize', 15)
    xlabel('$\phi_1(q)$', 'Interpreter', 'LaTex', 'Fontsize', 15)
    ylabel('$\phi_2(q)$', 'Interpreter', 'LaTex', 'Fontsize', 15)
    zlabel('$\phi_3(q)$', 'Interpreter', 'LaTex', 'Fontsize', 15)
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Reconstruct a Demonstration   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Color',[1 1 1])
% Select Demonstration
num = index(2);

% Reference Trajectories
q_ref = Data_train(1:qdim,1:num-1);
q_dot_ref = Data_train(qdim+1:2*qdim,1:num-1);

for dof=1:7
    phi_q = out_of_sample(q_ref', latent_mapping)';
    q_rec = zeros(size(q_ref));
    for j=1:length(phi_q)
        q_rec(:,j) = pinv(latent_mapping.M')*phi_q(:,j) + latent_mapping.mean';
    end    
    subplot(qdim,1,dof)
    for i=1:length(Qs)
        plot(q_ref(dof,:),'-','Color', [0 0 0], 'LineWidth',2); hold on;
        plot(q_rec(dof,:),'-.','Color',[1 0 0], 'LineWidth',2); hold on;
    end
    grid on;    
    title(sprintf('Demonstrated and PCA-Reconstructed Joint Positions for $q_%d$',dof), 'Interpreter', 'LaTex', 'Fontsize', 15)
    xlabel('Time (samples)','Interpreter', 'LaTex', 'Fontsize', 15)
    ylabel('Angle (rad)','Interpreter', 'LaTex', 'Fontsize', 15)
end
legend('Demonstration','PCA-Reconstructed')

figure('Color',[1 1 1])


% Select Demonstration
dt = 0.002;
q_dot_rec = [];
for dof=1:7    
    q_dot_rec_dof = (diff(q_rec(dof,:)')/0.02)';
    q_dot_rec = [q_dot_rec; q_dot_rec_dof];
end

for dof=1:7
    subplot(qdim,1,dof)
    for i=1:length(Qs)
        plot(q_dot_ref(dof,:),'-','Color', [0 0 0], 'LineWidth',2); hold on;
        plot(q_dot_rec(dof,:),'-.','Color',[1 0 0], 'LineWidth',2); hold on;
    end
    grid on;        
    title(sprintf('Demonstrated and PCA-Reconstructed Joint Velocities for $q_%d$',dof), 'Interpreter', 'LaTex', 'Fontsize', 15)
    xlabel('Time (samples)','Interpreter', 'LaTex', 'Fontsize', 15)
    ylabel('$\omega$ (rad/s)','Interpreter', 'LaTex', 'Fontsize', 15)
    
end
legend('Demonstration','PCA-Reconstructed')

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Compare executed motion with JT-DS vs Demonstrated one %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Color',[1 1 1])
qdim = size(Qs{1},1);

% Select Demonstration
num = index(2);
sample = 200;

% Reference Trajectories
q_ref = Data_train(1:qdim,1:num-1);
q_dot_ref = Data_train(qdim+1:2*qdim,1:num-1);

from_opt = 1;
if from_opt
    x_target = Data_train(end-8:end,1);
    q_rec = q_ref;
    q_dot_rec = compute_trajectory(motion_generator, q_ref, x_target, options.orientation_flag);
    
    q_rec_new = q_rec;
    for i=2:length(q_rec)       
        q_rec_new(:,i) = (q_rec(:,i-1) + q_dot_rec(:,i));
    end
    q_rec = q_rec_new;
else
    q_rec = pour_JTDS_q(:,1:sample:end);
    
    % Select Demonstration        
    dt = 0.002;
    q_dot_rec_ = [];
    for dof=1:7
        q_rec_ = pour_JTDS_q(dof,:)';
        q_dot_rec_derivs = sgolay_time_derivatives(q_rec_, dt, 2, 3, 15);
        q_dot_dof = q_dot_rec_derivs(:,:,2)';
        q_dot_rec_ = [q_dot_rec_; q_dot_dof];
    end
    q_dot_rec = q_dot_rec_(:,1:sample:end);        
    
end

for dof=1:7
    subplot(qdim,1,dof)
    plot(q_ref(dof,:),'-','Color', [0 0 0], 'LineWidth',2); hold on;
    plot(q_rec(dof,:),'-.','Color',[1 0 0], 'LineWidth',2); hold on;
    grid on;
    title(sprintf('Demonstrated and JT-DS Joint Positions for $q_%d$',dof), 'Interpreter', 'LaTex', 'Fontsize', 15)
    xlabel('Time (samples)','Interpreter', 'LaTex', 'Fontsize', 15)
    ylabel('Angle (rad)','Interpreter', 'LaTex', 'Fontsize', 15)
end
legend('Demonstration','JT-DS output')

figure('Color',[1 1 1])
qdim = size(Qs{1},1);


for dof=1:7
    subplot(qdim,1,dof)
    plot(q_dot_ref(dof,:),'-','Color', [0 0 0], 'LineWidth',2); hold on;
    plot(q_dot_rec(dof,:),'-.','Color',[1 0 0], 'LineWidth',2); hold on;
    grid on;
    title(sprintf('Demonstrated and JT-DS Joint Velocities for $q_%d$',dof), 'Interpreter', 'LaTex', 'Fontsize', 15)
    xlabel('Time (samples)','Interpreter', 'LaTex', 'Fontsize', 15)
    ylabel('$\omega$ (rad/s)','Interpreter', 'LaTex', 'Fontsize', 15)
end
legend('Demonstration','JT-DS output')

%% Plot First 6 Joint Position trajectories
figure('Color',[1 1 1])
subplot(1,2,1)
scatter3(q_ref(1,:),q_ref(2,:),q_ref(3,:),20,'*'); hold on;
scatter3(q_ref(1,end),q_ref(2,end),q_ref(3,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
scatter3(q_rec(1,:),q_rec(2,:),q_rec(3,:),20,'o','filled'); hold on;
scatter3(q_rec(1,end),q_rec(2,end),q_rec(3,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
axis tight
grid on;
title('Joint-Space positions', 'Interpreter', 'LaTex', 'Fontsize', 15)
xlabel('$\theta_1$', 'Interpreter', 'LaTex', 'Fontsize', 15)
ylabel('$\theta_2$', 'Interpreter', 'LaTex', 'Fontsize', 15)
zlabel('$\theta_3$', 'Interpreter', 'LaTex', 'Fontsize', 15)

subplot(1,2,2)
scatter3(q_ref(4,:),q_ref(5,:),q_ref(6,:),20,'*'); hold on;
scatter3(q_ref(4,end),q_ref(5,end),q_ref(6,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
scatter3(q_rec(4,:),q_rec(5,:),q_rec(6,:),20,'o','filled'); hold on;
scatter3(q_rec(4,end),q_rec(5,end),q_rec(6,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
axis tight
grid on;
title('Joint-Space positions', 'Interpreter', 'LaTex', 'Fontsize', 15)
xlabel('$\theta_4$', 'Interpreter', 'LaTex', 'Fontsize', 15)
ylabel('$\theta_5$', 'Interpreter', 'LaTex', 'Fontsize', 15)
zlabel('$\theta_6$', 'Interpreter', 'LaTex', 'Fontsize', 15)


%% Plot First 6 Joint Velocity trajectories
figure('Color',[1 1 1])
subplot(1,2,1)
scatter3(q_dot_ref(1,:),q_dot_ref(2,:),q_dot_ref(3,:),20,'*'); hold on;
scatter3(q_dot_ref(1,end),q_dot_ref(2,end),q_dot_ref(3,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
scatter3(q_dot_rec(1,:),q_dot_rec(2,:),q_dot_rec(3,:),20,'o','filled'); hold on;
scatter3(q_dot_rec(1,end),q_dot_rec(2,end),q_dot_rec(3,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
axis tight
grid on;
title('Joint-Space velocities', 'Interpreter', 'LaTex', 'Fontsize', 15)
xlabel('$\omega_1$', 'Interpreter', 'LaTex', 'Fontsize', 15)
ylabel('$\omega_2$', 'Interpreter', 'LaTex', 'Fontsize', 15)
zlabel('$\omega_3$', 'Interpreter', 'LaTex', 'Fontsize', 15)

subplot(1,2,2)
scatter3(q_dot_ref(4,:),q_dot_ref(5,:),q_dot_ref(6,:),20,'*'); hold on;
scatter3(q_dot_ref(4,end),q_dot_ref(5,end),q_dot_ref(6,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
scatter3(q_dot_rec(4,:),q_dot_rec(5,:),q_dot_rec(6,:),20,'o','filled'); hold on;
scatter3(q_dot_rec(4,end),q_dot_rec(5,end),q_dot_rec(6,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
axis tight
grid on;
title('Joint-Space velocities', 'Interpreter', 'LaTex', 'Fontsize', 15)
xlabel('$\omega_4$', 'Interpreter', 'LaTex', 'Fontsize', 15)
ylabel('$\omega_5$', 'Interpreter', 'LaTex', 'Fontsize', 15)
zlabel('$\omega_6$', 'Interpreter', 'LaTex', 'Fontsize', 15)

%% Playback Motions with Robot Siumulator

% For Training Trajectory
PlaybackTrajectory(robotplant, q_ref, dt);

%% For Executed Trajectory
PlaybackTrajectory(robotplant, q_rec, dt);

%% Compare Joint-Space Trajectories in Embedding
% Plot in 3D-space
figure('Color',[1 1 1])  
for i=1:length(Qs)
    phi_q_ref = out_of_sample(q_ref', latent_mapping)';
    scatter3(phi_q_ref(1,:),phi_q_ref(2,:),phi_q_ref(3,:),20,'*');
    
    phi_q_rec = out_of_sample(q_rec', latent_mapping)';
    scatter3(phi_q_rec(1,:),phi_q_rec(2,:),phi_q_rec(3,:),20,'o','filled');
    
    scatter3(phi_q_ref(1,end),phi_q_ref(2,end),phi_q_ref(3,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
    scatter3(phi_q_rec(1,end),phi_q_rec(2,end),phi_q_rec(3,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
    axis tight
end
grid on;
title('Joint Trajectories in PCA-space $\phi(q)$', 'Interpreter', 'LaTex', 'Fontsize', 15)
xlabel('$\phi_1(q)$', 'Interpreter', 'LaTex', 'Fontsize', 15)
ylabel('$\phi_2(q)$', 'Interpreter', 'LaTex', 'Fontsize', 15)
zlabel('$\phi_3(q)$', 'Interpreter', 'LaTex', 'Fontsize', 15)


%% Generate Task-Space Trajectories from Training Data
task_space_traj = zeros(9,length(q_ref));
for i=1:length(q_ref)
    trans_tmp=robotplant.robot.fkine(q_ref(:,i));
    task_space_traj(:,i) = [trans_tmp(1:3,1); trans_tmp(1:3,2); trans_tmp(1:3,end)];
end
x_target = task_space_traj(:,end);

% Extract Task-Space Trajectories from JTDS motion
task_space_JTDS =  pour_JTDS_x;
x_target_JTDS = task_space_JTDS(:,end);

% Plot Position trajectories
figure('Color',[1 1 1])
scatter3(task_space_traj(end-2,:),task_space_traj(end-1,:),task_space_traj(end,:),20,'*'); hold on;
scatter3(task_space_traj(end-2,end),task_space_traj(end-1,end),task_space_traj(end,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
scatter3(task_space_JTDS(end-2,:),task_space_JTDS(end-1,:),task_space_JTDS(end,:),20,'*'); hold on;
scatter3(task_space_JTDS(end-2,end),task_space_JTDS(end-1,end),task_space_JTDS(end,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
axis tight
grid on;
title('Task-Space trajectory (Position)', 'Interpreter', 'LaTex', 'Fontsize', 15)
xlabel('$x_1$', 'Interpreter', 'LaTex', 'Fontsize', 15)
ylabel('$x_2$', 'Interpreter', 'LaTex', 'Fontsize', 15)
zlabel('$x_3$', 'Interpreter', 'LaTex', 'Fontsize', 15)

% Plot Orientation trajectories
figure('Color',[1 1 1])
subplot(1,2,1)
scatter3(task_space_traj(1,:),task_space_traj(2,:),task_space_traj(3,:),20,'*'); hold on;
scatter3(task_space_traj(1,end),task_space_traj(2,end),task_space_traj(3,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
scatter3(task_space_JTDS(1,:),task_space_JTDS(2,:),task_space_JTDS(3,:),20,'*'); hold on;
scatter3(task_space_JTDS(1,end),task_space_JTDS(2,end),task_space_JTDS(3,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
axis tight
grid on;
title('Task-Space trajectory (Orientation, 1st Component)', 'Interpreter', 'LaTex', 'Fontsize', 15)
xlabel('$R_{1,1}$', 'Interpreter', 'LaTex', 'Fontsize', 15)
ylabel('$R_{1,2}$', 'Interpreter', 'LaTex', 'Fontsize', 15)
zlabel('$R_{1,3}$', 'Interpreter', 'LaTex', 'Fontsize', 15)
subplot(1,2,2)
scatter3(task_space_traj(4,:),task_space_traj(5,:),task_space_traj(6,:),20,'*'); hold on;
scatter3(task_space_traj(4,end),task_space_traj(5,end),task_space_traj(6,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
scatter3(task_space_JTDS(4,:),task_space_JTDS(5,:),task_space_JTDS(6,:),20,'*'); hold on;
scatter3(task_space_JTDS(4,end),task_space_JTDS(5,end),task_space_JTDS(6,end),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
axis tight
grid on;
title('Task-Space trajectory (Orientation, 2nd Component)', 'Interpreter', 'LaTex', 'Fontsize', 15)
xlabel('$R_{2,1}$', 'Interpreter', 'LaTex', 'Fontsize', 15)
ylabel('$R_{2,2}$', 'Interpreter', 'LaTex', 'Fontsize', 15)
zlabel('$R_{2,3}$', 'Interpreter', 'LaTex', 'Fontsize', 15)