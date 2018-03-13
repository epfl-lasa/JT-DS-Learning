%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     Learning JTDS Models (including orientation) on Different Datasets     %%                                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

if do_plots
    figure('Color',[1 1 1])    
    for i=1:length(Qs)
        Data_ = [];
        Data_ = Qs{i};
        subplot(1,2,1)
        scatter3(Data_(1,:),Data_(2,:),Data_(3,:),10,'filled'); hold on;
        xlabel('$q_1$','Interpreter','LaTex');ylabel('$q_2$','Interpreter','LaTex');zlabel('$q_3$','Interpreter','LaTex')
        title('First 3 Joint Angles (Raw)', 'Interpreter','LaTex')
        subplot(1,2,2)
        scatter3(Data_(4,:),Data_(5,:),Data_(6,:),10,'filled'); hold on;
        title('Last 3 Joint Angles (Raw)', 'Interpreter','LaTex')
        xlabel('$q_4$','Interpreter','LaTex');ylabel('$q_5$','Interpreter','LaTex');zlabel('$q_6$','Interpreter','LaTex')
    end    
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Train a JTDS model on the current dataset   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Choose Lower-Dimensional Mapping Technique
mapping = {'PCA'}; % 'None', 'PCA', 'KPCA'

%%% Learning options %%%
options = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% To remove orientation from target 
% simply set flag = 0 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
options.orientation_flag = 0; 
options.tol_cutting = 0.1;

%%% Dim-Red options %%%
options.explained_variance_threshold = .95;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% If choosen mapping is K-PCA you 
% need to choose the kernel width
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Split Dataset for Training/Testing        %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tt_ratio = 0.6;
train = round(length(Qs)*tt_ratio);
Qs_train = []; Ts_train = [];
Qs_test = [];   Ts_test = [];
rand_ids = [1:length(Qs)];
for ii=1:length(Qs)
    if ii < train
        Qs_train{ii,1} = Qs{rand_ids(ii)}; Ts_train{ii,1} = Ts{rand_ids(ii)};
    else
        Qs_test{ii-train+1,1} = Qs{rand_ids(ii)}; Ts_test{ii-train+1,1} = Ts{rand_ids(ii)};
    end
end

%%% Pre-process Data %%%
[Data_train, ~] = preprocess_demos_jtds(robotplant, Qs_train, Ts_train, options.tol_cutting,options.orientation_flag);
[Data_test, ~]  = preprocess_demos_jtds(robotplant, Qs_test, Ts_test, options.tol_cutting,options.orientation_flag);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Learn JTDS model for the Current Dataset      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

% Compute RMSE on testing data
rmse_test = mean(trajectory_error(motion_generator, Data_test(1:dimq, :), Data_test(dimq+1:2*dimq, :), Data_test(2*dimq+1:end, :),options.orientation_flag));
fprintf('Using %s mapping, got prediction RMSE on testing: %d \n', mapping_name, rmse_test);

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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Reproduce learned motion and compute tasks-space metrics %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Pre-process Data %%%
selected_demo = 1;
[demos_train_task] = preprocess_demos_seds(robotplant, Qs_train,options.orientation_flag)';
task_space_traj = demos_train_task{selected_demo}; 
x_target = task_space_traj(end-2:end,end);
figure('Color',[1 1 1])
scatter3(task_space_traj(end-2,:),task_space_traj(end-1,:),task_space_traj(end,:),20,'*'); hold on;
scatter3(x_target(1),x_target(2),x_target(3),100,'o','filled','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0]); hold on;
axis tight
grid on;
title('Task-Space trajectory', 'Interpreter', 'LaTex', 'Fontsize', 15)
xlabel('$x_1$', 'Interpreter', 'LaTex', 'Fontsize', 15)
ylabel('$x_2$', 'Interpreter', 'LaTex', 'Fontsize', 15)
zlabel('$x_3$', 'Interpreter', 'LaTex', 'Fontsize', 15)

% Simulate task-space trajectory with JT-DS
q_init = Data_train(1:dimq,1);
dt = 0.01;
while(1)
    
    % compute state of end-effector
    x = robot.fkine(q);
    x = x(1:2,4); 
    xd = robot.jacob0(q)*qd';    
    xd = xd(1:2) ;  
    
    %reference_vel(t);
    xd_ref = reshaped_ds(x-target);
    
    % put lower bound on speed, just to speed up simulation
    th = 1.0;
    if(norm(xd_ref)<th)
        xd_ref = xd_ref/norm(xd_ref)*th;
    end
    xdd_ref = -(xd - xd_ref)/dt*0.5;
    
    % Compute Damping Matrix
    Q = findDampingBasis(xd_ref);
    L = [2 0;0 8];     % inverse
    L = [4 0;0 4];   % icra-lfd-tutorial
    D = Q*L*Q';
    
    % Compute Cartesian Control    
    u_cart = - D*(xd-xd_ref);
    
    % feedforward term
    u_cart = u_cart + simple_robot_cart_inertia(robot,q)*xdd_ref;           
    
    % compute joint space control
    u_joint = robot.jacob0(q)'*[u_cart;zeros(4,1)];
    
    % apply control to the robot
    qdd = robot.accel(q,qd,u_joint')';
    
    % integrate one time step
    qd = qd+dt*qdd;
    q = q+qd*dt+qdd/2*dt^2;
    t = t+dt;
    if (norm(x - target)<0.01)
        break
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      If you are happy with the results, export the model       %%
%        for execution with the rtk-DS cpp class                 %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% All parameters in 1 file
% model_dir = strcat('./learned_JTDS_models/',choosen_dataset);
% mkdir(model_dir); 
% filename = strcat(model_dir,'/JTDS_model.txt')
% out = export2JSEDS_Cpp_lib(filename,Priors,Mu,Sigma,robot, As, latent_mapping.M);

% 1 file per paramater
model_dir = strcat('./learned_JTDS_models/',choosen_dataset);
mkdir(model_dir); 
cd(model_dir)
if strcmp(mapping_name, 'None')
    M_p = eye(7);
else
    M_p = latent_mapping.M';
end
out = export2JSEDS_Cpp_lib_v2(Priors, Mu, Sigma, As, latent_mapping.M', latent_mapping.mean, Data_train);

% save mat file of variables
M = latent_mapping.M';
save('model.mat','Priors','Mu','Sigma', 'As', 'M', 'Data_train', 'robotplant','latent_mapping')
