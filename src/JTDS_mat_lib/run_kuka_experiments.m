%% Train and compare a series of JTDS models
demos_location = '~/Downloads/fore_hand/data.mat';
fprintf('Loading demonstrations from %s \n', demos_location);
[Qs, Ts] = ImportDemonstrations(demos_location);
% If the data is very dense, initializing the semidefinite program may take
% a long time. In this case, it may help to thin down the number of
% demonstrated points (by varying "thinning_ratio", so long as there are still sufficient points to
% satisfactorily reconstruct the shape of the trajectory.
% In the KUKA case, we get 500 datapoints per second, so we recommend shrinking the data density considerably
thinning_ratio = 50;
for i = 1:length(Qs)
    Qs{i} = Qs{i}(:, 1:thinning_ratio:end);
    Ts{i} = Ts{i}(:, 1:thinning_ratio:end);
end
demos = Qs;
times = Ts;

mappings_to_compare = {'None', 'PCA', 'KPCA'};
options.kpca_sigma = 0.1;
options.thinning_ratio = 80;
options.GMM_sigma_type = 'full'; % Can be either 'full' or 'diagonal'
options.explained_variance_threshold = .90;
options.GMM_maximize_BIC = false;
options.fixed_num_gaussians = 5;
%options.max_gaussians = 8;
options.BIC_regularization = 2; % this quantity should probably be between 1 and 3
options.learn_with_bounds = false;
options.verbose = false;

for i = 1:size(mappings_to_compare, 2)
    mapping_name = mappings_to_compare{i};
    fprintf('Training JTDS generator using %s mapping...\n', mapping_name);
    options.latent_mapping_type = mapping_name;
    [motion_generator, resulting_rmse] = kuka_experiment(demos, times, options);
    fprintf('Using %s mapping, got prediction RMSE: %d \n', mapping_name, resulting_rmse);
end