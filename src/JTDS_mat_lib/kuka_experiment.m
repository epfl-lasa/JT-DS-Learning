function [motion_generator, rmse] = kuka_experiment(demos, times, options)
% This function is a wrapper that sets up and trains a JTDS model with a 
% given set of hyperparameters.
%
% Inputs -----------------------------------------------------------------
%
%
%   o demos:   A variable containing all demonstrations (only
%              joint trajectories). The variable 'demos' should follow the
%              following format:
%              - demos{n}: qdim x T^n matrix representing the qdim dimensional
%                          joint trajectories. T^n is the number of datapoints in
%                          each demonstration (1 < n < N)
%
%   o time:    This variable can be provided in two ways. If time steps
%              between all demonstration points are the same, 'time' could be
%              given as a positive scalar (i.e. time = dt). If not, 'time'
%              should follow the following format:
%              - time{n}: 1 x T^n vector representing the time array of length
%                         T^n cor
%
%   o options: This variable contains the options struct passed into
%              JTDS_Solver. For more information, call 'doc JTDS_Solver'.
%              One can specify additional options, including:
%              o options.tol_cutting - used to trim the ends of the
%                  demonstrations. For more info, call 
%                  'doc preprocess_demos_jts'. Default 0.1.
%              o options.robot - used to provide a custom robot. If not,
%                  given the default DH params for the KUKA LWR 4+.
%
% Outputs -----------------------------------------------------------------
%
%
%   o motion_generator: 
%               a MotionGenerator object (or MotionGeneratorBounded
%               object if options.learn_with_bounds = true) with parameters
%               learned through the JTDS optimization.
%   
%   o rmse:     The root mean-squared error between the true joint velocity
%               and the estimated joint velocity (using motion_generator)
%               evaluated pointwise at each location along the original
%               demos.


    if ~(isfield(options, 'tol_cutting'))
        options.tol_cutting = 0.1;
    end
    if ~(isfield(options, 'learn_with_bounds'))
        options.learn_with_bounds = true;
    end
    if ~(isfield(options, 'robot'))
        % DH parameters for the KUKA LWR 4+ robot
        dimq = 7;
        A = [0 0 0 0 0 0 0.05];
        Alpha = pi/2*[1 -1 -1 1 1 -1 0];
        D = [.34 0 .4 0 .4 0 .279];
        Qmin = 2*pi/180*[-85, -90, -100, -110, -140, -90, -120];
        Qmax = 2*pi/180*[85, 90, 100, 110, 140, 90, 120];
        % Create a model of the robot
        robot = initialize_robot(A,D,Alpha,Qmin,Qmax);
    else
        robot = options.robot;
    end
    % Create a model plant for the robot's motor controller
    robotplant = RobotPlant(robot, 'end_trans');
    [Data, ~] = preprocess_demos_jtds(robotplant, demos, times, options.tol_cutting);    
    
    
    [Priors, Mu, Sigma, As, latent_mapping] = JTDS_Solver(Data, robotplant, options);
    
    
    % Create a controller which uses the learned policy
    if options.learn_with_bounds
        motion_generator = MotionGeneratorBounded(robotplant, Mu, Sigma, Priors, As, latent_mapping);
    else
        motion_generator = MotionGenerator(robotplant, Mu, Sigma, Priors, As, latent_mapping);
    end
    rmse = mean(trajectory_error(motion_generator, Data(1:dimq, :), Data(dimq+1:2*dimq, :), Data(2*dimq+1:end, :)));
end

