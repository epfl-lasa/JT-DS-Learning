%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Evaluation of Learning Schemes for JTDS Models on Different Datasets     %%
%  Train and compare a series of JTDS models with different dimensionality  %%   
%  reduction schemes and GMM model fitting                                  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

% Visualize Raw Demonstrations in double 3D plot (ignoring 7d)
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find suitable range for rbf kernel (Kernel-PCA) %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Stack Demonstrations
X = [];
for i=1:length(Qs)
    X = [X; Qs{i}'];
end
% Compute Distances
[D, mean_D, max_D] = computePairwiseDistances(X,do_plots);

% Compute Maximum Feasible Sigma for RBF Kernel
max_rbf = max_D/sqrt(2);

fprintf('Maximum Scaled Euclidean Pairwise Distance %2.2f\n',max_rbf);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Search for the best KPCA Hyper-params for JTDS models on current dataset %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
%%% Type of DR Methods %%%
do_plots = 1;
mapping_name = 'KPCA';
rbf_range = logspace(log10(max_rbf/2),log10(max_rbf*5),10);

%%% Learning options %%%
options = [];
options.tol_cutting = 0.1;

%%% Dim-Red options %%%
options.explained_variance_threshold = .95;

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
%   Search for reasonable rbf range; i.e. whose    %
%    expected explained variance is < dimq         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tt_ratio = 0.6;
train = round(length(Qs)*tt_ratio);
Qs_train = []; Ts_train = [];
Qs_test = [];   Ts_test = [];
rand_ids = randsample(length(Qs),length(Qs))';
for ii=1:length(Qs)
    if ii < train
        Qs_train{ii,1} = Qs{rand_ids(ii)}; Ts_train{ii,1} = Ts{rand_ids(ii)};
    else
        Qs_test{ii-train+1,1} = Qs{rand_ids(ii)}; Ts_test{ii-train+1,1} = Ts{rand_ids(ii)};
    end
end
    
%%% Pre-process Data %%%
[Data_train, ~] = preprocess_demos_jtds(robotplant, Qs_train, Ts_train, options.tol_cutting);
[Data_test, ~]  = preprocess_demos_jtds(robotplant, Qs_test, Ts_test, options.tol_cutting);

% First compute Kernel PCA with M Components
kpca_num_dims = size(Data_train,2);
if do_plots; figure('Color',[1 1 1]); end;
p_s = [];
cumsums = [];
for k=1:length(rbf_range)
    [~, latent_mapping] = compute_mapping(Data_train', 'KPCA', kpca_num_dims, 'gauss', rbf_range(k));
    
    % Extract Eigenvalues
    L     = real(latent_mapping.L);
    
    % Compute Cumulative Explained Variance
    explained_variance = L./(sum(L));
    cum_sum = cumsum(explained_variance);
    cumsums = [cumsums; cum_sum'];
    p_s = [p_s sum(cum_sum < options.explained_variance_threshold)+1];
    
    % Plot Curve
    if do_plots
        c = [rand rand rand];
        plot(cum_sum,'-*','Color',c); hold on;
    end
end
if do_plots
    xlabel('Eigenvector Idx','Interpreter','Latex')
    ylabel('Explained Variance in Feature Space','Interpreter','Latex')
    legend(strread(num2str(rbf_range),'%s'))
    title('Initial Range of $\sigma$', 'Interpreter','LaTex')
    grid on;    
    axis tight    
    for k=1:length(p_s)
        scatter(p_s(k),cumsums(k,p_s(k)),50,[1 0 0]); hold on
    end    
end


% Truncate the range to values whose exp. var < dimq
new_rbf_range      = rbf_range(p_s <= dimq);
adjusted_rbf_range = logspace(log10(new_rbf_range(1)),log10(new_rbf_range(end)),10)

display('Press ENTER to continue with this range:');
pause;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Learn JTDS Model with different methods and parameters %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
repetitions = 10;
K_s         = zeros(repetitions,length(rbf_range));
rmse_train  = zeros(repetitions,length(rbf_range));
rmse_test   = zeros(repetitions,length(rbf_range));
kpca_dims   = zeros(repetitions,length(rbf_range));
for j = 1:repetitions
    Qs_train = []; Ts_train = [];
    Qs_test = [];   Ts_test = [];
    rand_ids = randsample(length(Qs),length(Qs))';
    for ii=1:length(Qs)
        if ii < train
            Qs_train{ii,1} = Qs{rand_ids(ii)}; Ts_train{ii,1} = Ts{rand_ids(ii)};
        else
            Qs_test{ii-train+1,1} = Qs{rand_ids(ii)}; Ts_test{ii-train+1,1} = Ts{rand_ids(ii)};
        end
    end
    
    %%% Pre-process Data %%%
    [Data_train, ~] = preprocess_demos_jtds(robotplant, Qs_train, Ts_train, options.tol_cutting);
    [Data_test, ~]  = preprocess_demos_jtds(robotplant, Qs_test, Ts_test, options.tol_cutting);
                     
    fprintf('********************* %d Fold *****************************\n', j);
    
    for k = 1:length(adjusted_rbf_range)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %       Learn JTDS variants for current fold      %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        options.latent_mapping_type = mapping_name;
        options.kpca_sigma = adjusted_rbf_range(k);
        fprintf('Training JTDS generator using %s mapping with sigma=%2.2f\n', options.latent_mapping_type, options.kpca_sigma);
                
        % Run JTDS Solver function        
        [Priors, Mu, Sigma, As, latent_mapping] = JTDS_Solver_v2(Data_train, robotplant, options);
        K_s(j,k) = length(Priors);       
        
        
        % Collect Number of dimensions
        kpca_dims(j,k) = length(latent_mapping.L);        
        
        % Generate Trajectories from Learnt JTDS
        motion_generator = MotionGenerator(robotplant, Mu, Sigma, Priors, As, latent_mapping);
        
        % Compute RMSE on training data
        rmse_train(j,k) = mean(trajectory_error(motion_generator, Data_train(1:dimq, :), Data_train(dimq+1:2*dimq, :), Data_train(2*dimq+1:end, :)));
        fprintf('Using %s mapping, got prediction RMSE on training: %d \n', mapping_name, rmse_train(j,k));
        
        % Compute RMSE on testing data
        rmse_test(j,k) = mean(trajectory_error(motion_generator, Data_test(1:dimq, :), Data_test(dimq+1:2*dimq, :), Data_test(2*dimq+1:end, :)));
        fprintf('Using %s mapping, got prediction RMSE on testing: %d \n', mapping_name, rmse_test(j,k));
    end
    fprintf('*************************************************************\n');
end

