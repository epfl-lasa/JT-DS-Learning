function [Priors, Mu, Sigma, As, latent_mapping] = JTDS_Solver(Data, robotplant, options)
%JTDS_LEARNING 
% This function takes as input a set of demonstrations and a kinematic
% system, and outputs parameters for a Joint-space Task-oriented Dynamical
% System model. 

% Inputs -----------------------------------------------------------------
%
%   o Data:    A (2dimq + dimx) x N_Total matrix containing all demonstration data points.
%              The rows are assembled as follows:
%              1:dimq - joint angle values throughout the trajectories
%              dimq+1:2dimq - first time derivative of joint angles
%              2dimq+1:2dimq+dimx - target task position values
%              Each column of Data stands
%              for a datapoint. All demonstrations are put next to each other 
%              along the second dimension. For example, if we have 3 demos
%              D1, D2, and D3, then the matrix Data is:
%                               Data = [D1 D2 D3]
%              Not that this Data matrix is precisely the type outputted by
%              preprocess_demos.m
%
%   o options: A structure to set the optional parameters of the
%               joint-specific optimization. The options are specified
%               below.
%
%
%   ------------------- OUTPUTS ------------------
%   o Priors: 1 x k, the priors of the GMM which encodes the spatial significance of each behavior
%
%   o Mu: qdim x k, the means of the GMM which encodes the spatial significance of each behavior
%
%   o Sigma: qdim x qdim x k, the covariance matrices of each component of 
%               the GMM which encodes the spatial significance of each behavior
%
%   o As: qdim x qdim x k, the augmentation matrices defining each local behavior
%
%   o latent_mapping: a container, in the style of drtoolbox, for a 
%               dimensionality reduction technique mapping a joint-space 
%               position to alatent-space position
%
%
%  ---------------------OPTIONS-------------------
%
%   o options.latent_mapping_type: the type of function used to reduce
%           the dimensionality of the joint space, mapping from q to z.
%           Must be one of:
%               'PCA' - Principle Component Analysis
%               'KPCA' - Kernel PCA with Gaussian kernel w/ variance 1 
%               'Autoencoder' - Multi-layer deep denoising autoencoder
%               'None' - Apply no dimensionality reduction technique.
%           Default is PCA.
%
%   o options.explained_variance_threshold: the pct (between 0 and 1) of
%           training data variance that must be explained by the latent
%           transform, if the latent transform is principle-component
%           based (either PCA or KPCA). Default is 0.98 .
%
%   o options.autoencoder_num_dims: the size of the final encoding layer of
%           the autoencoder, only applicable if latent_mapping_type =
%           'Autoencoder'. Default 2.
%
%   o options.kpca_sigma: the sigma value to be used in the Gaussian
%           kernels if using KPCA dimensionality reduction.
%           Default 1.0
%
%   o options.GMM_sigma_type: the shape of the covariance matrix of the GMM
%           components. Can be either 'full' or 'diagonal' - 'full allows for
%           better fitted Gaussians, but 'diagonal' allows for faster
%           convergence in higher-dimensional systems. Default 'full'.
%
%   o options.maximize_BIC: if true, tries different GMMs with between 1
%           and options.max_gaussians components, and chooses the one with 
%           the highest BIC (given some regularization).
%           If false, fits a GMM with exactly options.fixed_num_gaussians
%           components. Default = true.
%
%   o options.max_gaussians: the maximum number of Gaussians to try in a
%           GMM. Must be specified if options.maximize_BIC = true. Default
%           = 8.
%
%   o options.fixed_num_gaussians: the precise number of Gaussians to use in a GMM.
%           Required if maximize_BIC = true
%
%   o options.GMM_num_replicates: number of different GMMs to try to fit
%           for each # components k. The higher this number, the longer it
%           takes to find an optimal GMM, but the more likely the procedure is
%           to have found the best possible GMM to describe the demonstrated
%           data. Default = 50
%
%   o options.BIC_regularization: a regularization parameter for ensuring
%           that GMMs with more components are penalized relative to GMMs with
%           fewer components, to make simpler (lower component) models more
%           desirable. Gigher component BIC must be smaller than lower-component BIC by 
%           a factpr of options.BIC_regularization^(-1*sign(old_BIC))
%           Default = 2.  (Recommended between 1 and 3)
%
%   o options.verbose: if true, prints helpful messages about current
%           progress of the learning algorithm. Default true.
%
%   o options.learn_with_bounds: if true, incorporates boundary avoidance
%           and joint limits into the motion policy (by computing S from the
%           paper). If false, ignores the limits, setting S = eye(dimq).
%
% 
%

    % Ensure all the options are initialized correctly:
    if ~(isfield(options, 'latent_mapping_type'))
        options.latent_mapping_type = 'PCA';
    end
    if ~(isfield(options, 'explained_variance_threshold'))
        options.explained_variance_threshold = 0.98;
    end
    if ~(isfield(options, 'kpca_sigma'))
        options.kpca_sigma = 1.0;
    end
    if ~(isfield(options, 'autoencoder_num_dims'))
        options.autoencoder_num_dims = 2;
    end
    if ~(isfield(options, 'GMM_sigma_type'))
        options.GMM_sigma_type = 'diagonal';
    end
    if ~(isfield(options, 'GMM_maximize_BIC'))
        options.GMM_maximize_BIC = true;
    end
    if ~(isfield(options, 'max_gaussians'))
        options.max_gaussians = 8;
    end
    if ~(options.GMM_maximize_BIC)
        % must have a specified number of Gaussians
        if ~(isfield(options, 'fixed_num_gaussians'))
            error('If options.GMM_maximize_BIC is false, must define the number of Gaussians used in GMM options.fixed_num_gaussians');
        end
    end     
    if ~(isfield(options, 'GMM_num_replicates'))
        options.GMM_num_replicates = 50;
    end
    if ~(isfield(options, 'BIC_regularization'))
        options.BIC_regularization = 2;
    end
    if ~(isfield(options, 'GMM_verbose'))
        options.GMM_verbose = true;
    end
    if ~(isfield(options, 'learn_with_bounds'))
        options.learn_with_bounds = true;
    end
    if ~(isfield(options, 'GMM_use_Qd'))
        options.GMM_use_Qd = false;
    end
    if options.GMM_use_Qd && ~(strcmp(options.latent_mapping_type, 'PCA') || strcmp(options.latent_mapping_type, 'None'))
        error('Cant use Qd in GMM unless using linear transform because cant marginalize away Qd for nonlinear methods')
    end
    
    % Extract the basic parameters of our system
    dimq = robotplant.robot.n;
    dimx = robotplant.dimx;
    n = size(Data, 2); % total number of demonstrated examples
    
    Q = Data(1:dimq, :); % joint positions
    Qd = Data(dimq + 1: 2*dimq, :); % joint velocities
    Xt = Data(2*dimq + 1:end, :); % target task positions
    
    if options.GMM_use_Qd
        TrainingData = [Q' Qd'];
    else
        TrainingData = Q';
    end    
    % Embed the joint space data into a lower-dimensional latent space
    % using a chosen method.
    switch(options.latent_mapping_type)
        case ('PCA')%, 'KPCA')
            [TransformedTrainingData, latent_mapping] = compute_mapping(TrainingData, 'PCA', options.explained_variance_threshold);
            dimz = size(latent_mapping.M, 2);
        case 'KPCA'
            [TransformedTrainingData, latent_mapping] = compute_mapping(TrainingData, 'KPCA', options.explained_variance_threshold, 'gauss', options.kpca_sigma);
            dimz = size(latent_mapping.V, 2);
        case 'Autoencoder'
            max_iter = 100;
            lambda = 0.001;
            [TransformedTrainingData, latent_mapping] = compute_mapping(TrainingData, 'Autoencoder', options.autoencoder_num_dims, lambda, max_iter);
            dimz = options.autoencoder_num_dims;
        case 'None'
            [TransformedTrainingData, latent_mapping] = compute_mapping(TrainingData, 'None');
            dimz = dimq;
        otherwise
            error('Incorrectly specified a latent_mapping type. Was %s, but must be one of "PCA", "KPCA", "Autoencoder", or "None"', options.latent_mapping_type);
    end
            
    if options.verbose
        disp(sprintf('Converted demonstrations into %d-D latent space using %s.', dimz, options.latent_mapping_type));
    end
    
    % Fit the embedded data to a GMM
    best_BIC = inf;
    if options.verbose
        disp('Fitting GMM to the demonstration data...');
    end
    if options.GMM_maximize_BIC % if we are determining the number of clusters K
        for j = 1:options.max_gaussians
            k_tmp = j;
            warning('off', 'all'); % there are a lot of really annoying warnings when fitting GMMs
            GMM_full_tmp = fitgmdist(TransformedTrainingData, k_tmp, 'Start', 'plus', 'CovarianceType',options.GMM_sigma_type, 'Regularize', .000001, 'Replicates', options.GMM_num_replicates); %fit a GMM to our data
            % Note that 'Regularize' is set to .000001 because that was
            % empirically determined to cause fairly stable convergence.
            % This parameter prevents the GMoptionsM from building clusters with
            % very few points in them.
            warning('on', 'all');
            Priors_tmp = GMM_full_tmp.ComponentProportion;
            Mu_tmp = transpose(GMM_full_tmp.mu(:, 1:dimz));

            if strcmp(options.GMM_sigma_type, 'full')
                Sigma_tmp = GMM_full_tmp.Sigma(1:dimz, 1:dimz, :);
            elseif strcmp(options.GMM_sigma_type, 'diagonal')
                for i = 1:k_tmp
                    Sigma_tmp(:,:,i) = diag(GMM_full_tmp.Sigma(1,1:dimz, i));
                end
            else
                error('Incorrectly specified GMM covariance type.');
            end
            % We compute the likelihood of this GMM (with this number of modes
            % k)
            bic_tmp = GMM_BIC(transpose(TransformedTrainingData), ones(size(TransformedTrainingData, 1), 1), Priors_tmp, Mu_tmp, Sigma_tmp, options.GMM_sigma_type(1:4));
            if bic_tmp < best_BIC*options.BIC_regularization^(-1*sign(best_BIC)); % If the GMM has a lower BIC score, this is our new best GMM
                % Note that to ensure each new Gaussian makes a significant improvement
                % to our model, we add regularization - namely that the new 
                % BIC must improve on the old one by a factor of at least 
                % options.BIC_regularization
                best_BIC = bic_tmp;
                k = k_tmp;
                Priors = Priors_tmp;
                Mu = Mu_tmp;
                Sigma = Sigma_tmp;
                if options.verbose
                    disp(sprintf('Found a GMM with %d modes and BIC = %d', k, best_BIC));
                end
            end
        end
    else % If we're using a pre-defined number of Gaussians options.fixed_num_gaussians
        k = options.fixed_num_gaussians;
        warning('off', 'all'); % there are a lot of really annoying warnings when fitting GMMs
        GMM_full = fitgmdist(TransformedTrainingData, options.fixed_num_gaussians, 'Start', 'plus', 'CovarianceType',options.GMM_sigma_type, 'Regularize', .000001, 'Replicates', options.GMM_num_replicates); %fit a GMM to our data
        warning('on', 'all');
        Priors = GMM_full.ComponentProportion;
        Mu = transpose(GMM_full.mu(:, 1:dimz));

        if strcmp(options.GMM_sigma_type, 'full')
            Sigma = GMM_full.Sigma(1:dimz, 1:dimz, :);
        elseif strcmp(options.GMM_sigma_type, 'diagonal')
            for i = 1:k
                Sigma(:,:,i) = diag(GMM_full.Sigma(1,1:dimz, i));
            end
        else
            error('Incorrectly specified GMM covariance type.');
        end
    end
    
    if options.GMM_use_Qd % Make sure to throw away Qd GMM portions
        latent_mapping.M = latent_mapping.M(1:dimq, :);
        latent_mapping.mean = latent_mapping.mean(1:dimq);
        Mu = Sigma(1:dimz, :);
        Sigma = Sigma(1:dimz, 1:dimz, :);
    end
    
    % Now that we have the GMM parameters, we need to find As to best fit
    % the data. To do this, we set up a YALMIP semidefinite optimization.
    
    if options.verbose
       disp('Setting up SDP optimization of A. This may take a while.');
    end
    
    sdp_options = sdpsettings('verbose', options.verbose);
    Constraints = [];
    
    if options.verbose
       disp('Ensure all A matrices are PSD.');
    end
    
    % Ensure all A matrices are PSD
    for i = 1:k
        A_vars{i} = sdpvar(dimq, dimq, 'symmetric');
        Constraints = [Constraints A_vars{i} >= 0];
    end
    
    if options.verbose
       disp('Finding weights for each behavior.');
    end
    tic;
    % Find the weighting for each behavior at each point "h", computed using
    % the normalized value of the GMM mixtures.
    Qd_basis = zeros(size(Qd));
    Ss = zeros(dimq, dimq, n);
    for i = 1:n
        q = Q(:, i); xt = Xt(:, i);
        Qd_basis(:, i) = robotplant.qd_basis(q, xt);
        if options.learn_with_bounds % if false, ignore joint limits
            Ss(:,:,i) = robotplant.compute_S(q);% the S value for each point, converted to a vector
        else
            Ss(:,:,i) = eye(dimq);
        end
        for j = 1:k % Compute the value of a single component of the GMM
            h_raw(j, i) = Priors(j).*gaussPDF(out_of_sample(q', latent_mapping)', Mu(:, j), Sigma(:, :, j));
        end
        htotal = sum(h_raw(:,i));
        if htotal == 0 % if our point is too far from any of the Gaussians, weight them equally
            h(:,i) = 1./k;
        else
            h(:, i) = h_raw(:,i)/htotal; % otherwise normalize the weight of each component
        end
    end
    % Calculate our estimated joint velocities caused by each local behavior
    Qd_approximated_raw = sdpvar(k,n, dimq, 'full');%zeros(size(Qd));
    for j = 1:k
        for i = 1:n
            % This is the overall JT-DS expression, combining all previous
            % parts
            Qd_approximated_raw(j,i, :) = Ss(:,:,i)*(h(j, i)*A_vars{j})*Ss(:,:,i)'*Qd_basis(:, i);
        end
    end
    toc;
    
    % Sum each of the local behaviors to generate the overall behavior at
    % each point
    Qd_approximated = sdpvar(dimq, n, 'full');
    for i = 1:n
        for j = 1:dimq
            Qd_approximated(j,i) = sum(Qd_approximated_raw(:,i,j));
        end
    end
    
    % Then calculate the difference between estimated joint velocity (i.e.
    % behavior) h*A*qd_basis with the true qd from demonstrations
    Qd_error = Qd_approximated - Qd;
    Qd_total_error = sdpvar(1,1); Qd_total_error(1,1) = 0;
    for i = 1:n
        Qd_total_error = Qd_total_error + norm(Qd_error(:, i));
    end
    Objective = Qd_total_error;

    
    % Now minimize this erro to find a best-fitting set of behavior
    % matrices
    if options.verbose
       disp('Solving SDP to find optimal A matrices...');
    end
    sol = optimize(Constraints, Objective, sdp_options);
    if sol.problem ~= 0
        yalmiperror(sol.problem);
    end
    for i = 1:k
        As(:,:,i) = value(A_vars{i});
    end
    if options.verbose
        fprintf('Total error: %d', value(Objective));
        disp('JSEDS Optimization complete.');
    end
end

