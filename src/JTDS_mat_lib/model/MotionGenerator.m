classdef MotionGenerator
    %MOTIONGENERATOR The class that contains the JT-DS dynamical system,
    %though without built-in joint limit functionality. For a controller
    %that also integrates joint limits, see the child class
    %"MotionGeneratorBounded".
    
    properties
        plant % a RobotPlant used to compute kinematics properties
        Mu % the means of the learned behavior GMM
        Sigma % the sigmas of the GMM
        Priors % the scaling of relative Gaussians of the GMM
        As % the augmentation matrices of the components of the GMM
        latent_mapping % the container for the details of the function mapping from the joint space q to its latent embedding space z
    end
    
    methods
        function obj = MotionGenerator(robotplant, mu, sigma, priors, A, latent_mapping)
            obj.plant = robotplant;
            if nargin == 1 % if only given the robot, create a default controller with no learned values
                dimq = robotplant.robot.n;
                obj.Mu = zeros(dimq, 1);
                obj.Sigma = eye(dimq);
                obj.Priors = 1;
                obj.As = eye(4);
                [~, obj.latent_mapping] = compute_mapping(eye(q), 'None');
            else
                obj.Mu = mu;
                obj.Sigma = sigma;
                obj.Priors = priors;
                obj.As = A;
                obj.latent_mapping = latent_mapping;
            end
        end
        
        function A = compute_A(obj, q)
            % This function computes the weighted A matrix based on the current
            % robot configuration. This combines the contributions of each of
            % the local augmentation matrices (As).
            k = length(obj.Priors);
            for i = 1:k
                h_raw(i) = obj.Priors(i)*mvnpdf(out_of_sample(q', obj.latent_mapping)', obj.Mu(:, i), obj.Sigma(:, :, i));
            end
            htotal = sum(h_raw);
            if htotal == 0 % if our point is too far from any of the Gaussians, weight them equally
                h = 1./k*ones(k,1);
            else
                h = h_raw/htotal;
            end
            A_total = zeros(size(obj.As(:,:,1)));
            for i = 1:k
                A_total = A_total + h(i)*obj.As(:, :, i);
            end
            A = A_total;
        end
        
        function qd = get_next_motion(obj, q, xt, ~) % Generates the next joint velocity given the joint position,
            % desired position, a GMM model, and an augmentation matrix for
            % each Gaussian. Does not do bound-checking.
            qd_basis = obj.plant.qd_basis(q, xt);
            qd = obj.compute_A(q)*qd_basis;
        end
        
        function  qd = get_next_motion_orientation(obj, q, xt, ~) % Generates the next joint velocity given the joint position,
            % desired position, a GMM model, and an augmentation matrix for
            % each Gaussian. Does not do bound-checking.
            qd_basis = obj.plant.qd_basis_orientation(q, xt);
            qd = obj.compute_A(q)*qd_basis;
        end
        
        
        function qd_fun = ODE_fun(obj, xt, dt,orientation_flag) % Yields a function handle
            % which computes the first order differential equation
            % associated with this system, i.e. f where qd = f(q, t)
            % (dt is a placeholder for a child class, and can be ignored in
            % this case)
            % Used when solving the system using an ODE solver.
            
            if orientation_flag==1
                qd_fun = @(t, q) obj.compute_A(q)*obj.plant.qd_basis_orientation(q, xt);
            else
                qd_fun = @(t, q) obj.compute_A(q)*obj.plant.qd_basis(q, xt);
            end
        end
        
        function options = ODE_options(obj, xt, goal_tolerance,orientation_flag) % provides an options struct
            % for the ODE solver to optionally use, to add useful
            % functionality to the ODE
            function [value, is_terminal, direction] = event_fun(t, y)
                value = zeros(size(t));
                for i = 1:length(t)
                    if orientation_flag==1
                        value(i) = norm(obj.plant.end_pos_orien(y(:, i)') - xt) - goal_tolerance;
                    else
                        value(i) = norm(obj.plant.forward_kinematics(y(:, i)') - xt) - goal_tolerance;
                    end
                end
                is_terminal = ones(size(t));
                direction = -1*ones(size(t));
            end
            options = odeset('Events', @event_fun, 'AbsTol', 1e-8);
        end
    end
    
end

