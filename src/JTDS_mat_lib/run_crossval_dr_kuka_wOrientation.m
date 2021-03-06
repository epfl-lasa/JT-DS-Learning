%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Evaluation of Learning Schemes for JTDS Models (including orientation) on Different Datasets     %%
%  Train and compare a series of JTDS models with different dimensionality                          %%   
%  reduction schemes and GMM model fitting                                                          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;
do_plots  = 1;
data_path = '../../Data/mat/'; % <-Insert path to datasets folder here
choosen_dataset = 'singularity'; % Options: 'back','fore','pour','pour_obst','foot','singularity';

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
%%  Train different JTDS models on current dataset %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Orientation is included

%%% Type of DR Methods %%%
mappings_to_compare = {'None', 'PCA'};

%%% Learning options %%%
options = [];
options.orientation_flag=1;
options.tol_cutting = 0.1;

%%% Dim-Red options %%%
options.explained_variance_threshold = .95;
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Learn JTDS Model with different methods and parameters %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
repetitions = 10;
K_s         = zeros(size(mappings_to_compare, 2),repetitions);
rmse_train  = zeros(size(mappings_to_compare, 2),repetitions);
rmse_test   = zeros(size(mappings_to_compare, 2),repetitions);
pca_dims    = zeros(1,repetitions);
kpca_dims   = zeros(1,repetitions);

for j = 1:repetitions        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %       Split Dataset for Training/Testing        %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tt_ratio = 0.8;
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
    [Data_train, ~] = preprocess_demos_jtds(robotplant, Qs_train, Ts_train, options.tol_cutting,options.orientation_flag);
    [Data_test, ~]  = preprocess_demos_jtds(robotplant, Qs_test, Ts_test, options.tol_cutting,options.orientation_flag);
    
    fprintf('********************* %d Fold *****************************\n', j);
    
    for i = 1:size(mappings_to_compare, 2)    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %       Learn JTDS variants for current fold      %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        mapping_name = mappings_to_compare{i};
        fprintf('Training JTDS generator using %s mapping...\n', mapping_name);
        options.latent_mapping_type = mapping_name;

        % Run JTDS Solver function
        [Priors, Mu, Sigma, As, latent_mapping] = JTDS_Solver_v2(Data_train, robotplant, options);
        K_s(i,j) = length(Priors);
        
        if strcmp('PCA',latent_mapping.name)
            pca_dims(1,j) = length(latent_mapping.lambda);
        end

        % Generate Trajectories from Learnt JTDS
        motion_generator = MotionGenerator(robotplant, Mu, Sigma, Priors, As, latent_mapping);
        
        % Compute RMSE on training data
        rmse_train(i,j) = mean(trajectory_error(motion_generator, Data_train(1:dimq, :), Data_train(dimq+1:2*dimq, :), Data_train(2*dimq+1:end, :),options.orientation_flag));
        fprintf('Using %s mapping, got prediction RMSE on training: %d \n', mapping_name, rmse_train(i,j));
        
        % Compute RMSE on testing data
        rmse_test(i,j) = mean(trajectory_error(motion_generator, Data_test(1:dimq, :), Data_test(dimq+1:2*dimq, :), Data_test(2*dimq+1:end, :),options.orientation_flag));
        fprintf('Using %s mapping, got prediction RMSE on testing: %d \n', mapping_name, rmse_test(i,j));
    end
    fprintf('*************************************************************\n');
end
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Print Statistics from Results           %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:size(mappings_to_compare, 2)
    mapping_i = mappings_to_compare{i};
    for j = i+1:size(mappings_to_compare, 2)
        mapping_j = mappings_to_compare{j};
        fprintf('========= Using %s instead of %s ===============\n', mapping_j, mapping_i);
        ei_train = rmse_train(i, :);
        ej_train = rmse_train(j, :);
        fprintf('Mean pct improvement on training set: %2.1f%% \n', mean(2*(ei_train-ej_train)./(ei_train+ej_train))*100);
        ei = rmse_test(i, :);
        ej = rmse_test(j, :);
        fprintf('Mean improvement on test set: %d \n', mean(ei-ej));
        fprintf('Mean pct improvement on test set: %2.1f%% \n', mean(2*(ei-ej)./(ei+ej))*100);
    end
end

