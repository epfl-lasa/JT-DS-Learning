classdef MotionGeneratorBounded < MotionGenerator
    %MOTIONGENERATOR The class which implements the dynamical system
    %specified using our model, and also ensures the system's joint limits are
    %respected.
    %When given the parameters of the system, can be used to generate the
    %next instantaneous velocity from a current position.
    
    properties
        Qmin % the minimum and maximum angles
        Qmax
    end
    
    methods
        
        function obj = MotionGeneratorBounded(robotplant, mu, sigma, priors, A, latent_mapping)
            obj@MotionGenerator(robotplant, mu, sigma, priors, A, latent_mapping);
            qlim = robotplant.robot.qlim;
            obj.Qmin = zeros(size(qlim, 1),1);
            obj.Qmax = zeros(size(qlim, 1),1);
            for i = 1:size(qlim, 1)
                obj.Qmin(i) = qlim(i,1);
                obj.Qmax(i) = qlim(i,2);
            end
        end
        
        function qd = get_next_motion(obj, q_old, xt, dt,orientation_flag) % Generates the next joint velocity given the joint position,
            % desired position, a GMM model, and an augmentation matrix for
            % each Gaussian. dt is the timestep for our next movement
            % We compute the velocity by integrating our joint limitation
            % matrix "S" into our classic MotionGenerator controller, all
            % the while ensuring positive-semidefiniteness.
            S = obj.plant.compute_S(q_old);
            if orientation_flag==1
                qd = S*obj.compute_A(q_old)*S'*obj.plant.qd_basis_orientation(q_old, xt);
            else
                qd = S*obj.compute_A(q_old)*S'*obj.plant.qd_basis(q_old, xt);
            end
        end
        
        function qd_fun = ODE_fun(obj, xt, dt,orientation_flag)  % Yields a function handle
            % which computes the first order differential equation
            % associated with this system, i.e. f where qd = f(t, q)
            % Used as input to an ODE solver.
            function qd = qd_fun_tmp(t, q)
                S = obj.plant.compute_S(q);
                if orientation_flag==1
                    qd = S*obj.compute_A(q)*S'*obj.plant.qd_basis_orientation(q, xt);
                else
                    qd = S*obj.compute_A(q)*S'*obj.plant.qd_basis(q, xt);
                end
            end
            qd_fun = @qd_fun_tmp;
        end
    end
    
end


