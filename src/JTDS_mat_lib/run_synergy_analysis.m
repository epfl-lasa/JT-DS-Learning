%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Evaluation of Learning Schemes for JTDS Models (including orientation) on Different Datasets     %%
%  Train and compare a series of JTDS models with different dimensionality                          %%   
%  reduction schemes and GMM model fitting                                                          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;
do_plots  = 1;
data_path = '../../Data/mat/'; % <-Insert path to datasets folder here
choosen_dataset = 'back'; % Options: 'back','fore','pour','pour_obst','foot','singularity';

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
[Data_train, ~] = preprocess_demos_jtds(robotplant, Qs, Ts, options.tol_cutting,options.orientation_flag);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Reconstruct a Demonstration   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Color',[1 1 1])
Data_ = [];
qdim = size(Qs{1},1);
for dof=1:7
    q_ref = Qs{i};
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
    title(sprintf('Raw and Reconstructed Demonstrations for $q_%d$',dof), 'Interpreter', 'LaTex', 'Fontsize', 15)
    xlabel('Time (samples)','Interpreter', 'LaTex', 'Fontsize', 15)
    ylabel('Angle (rad)','Interpreter', 'LaTex', 'Fontsize', 15)
end
legend('Raw','Reconstructed')


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Compare executed motion with JT-DS vs Demonstrated one %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 0.1; % the timestep
q_initial = [-0.728371429443359;-1.3605418920517;2.69252680319093;0.620675325393677;0.955035626888275;0.141930669546127;-0.17979271709919];
x_targets = [[-.5; -.5; 0.3]];
max_trajectory_duration = 60; % How long to interpolate the trajectory
goal_tolerance = 0.05; % How far away from the goal we need to get to accept





