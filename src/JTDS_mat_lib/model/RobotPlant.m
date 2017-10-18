classdef RobotPlant
    %ROBOTPLANT A class that encodes the robot's kinematic model, and also
    % for generating various convenient properties relating to the
    % kinematic model.
    
    properties
        robot % a Corke robotics-toolbox type robot to generate kinematics
        forward_kinematics % function that maps a joint configuration q to a task position x
        jacobian % takes as input q and gives corresponding dx/dq as output
        dimx % the dimension of x
    end
    
    methods
        function obj = RobotPlant(robot, task_type, custom_forward_kinematics, custom_jacobian, dimx)
            % the task-space description, task_type can be one of the following:
            % 'end_trans' = end-effector xyz coordinates, or 'custom'
            % if 'custom' must specify the forward kinematics and custom
            % jacobian using the 3rd and 4th arguments
            % dimx is the dimensionality of the task space
            % custom_qd_basis takes as input the object, q, and dt,
            % and outputs the basis for our velocity policy
            obj.robot = robot;
                
            if nargin<5 % If the number of arguments suggests standard non-custom kinematics
                if strcmp(task_type, 'end_trans')
                    obj.forward_kinematics = @obj.end_trans_forward_kinematics;
                    obj.jacobian = @obj.end_trans_jacobian;
                    obj.dimx = 3;
                else
                    error('Custom kinematics/jacobian functions were not specified');
                end
            else
                obj.forward_kinematics = custom_forward_kinematics;
                obj.jacobian = custom_jacobian;
                obj.dimx = dimx;
            end
        end
        
        function basis = qd_basis(obj, q, xt)
            % Computes the base joint velocity to move at a joint position q
            % toward a task position xt using the jacobian transpose
            % method (i.e. qd = J^T(H(q) - xt) ).
            % In general, this will be multiplied by an A matrix to get the
            % augmented joint velocity.
            basis = -1*transpose(obj.jacobian(q))*(obj.forward_kinematics(q) - xt);
        end
        
        function x = end_trans_forward_kinematics(obj,q)
            T = obj.robot.fkine(q);
            x = T(1:3, 4);
        end
        
        function j = end_trans_jacobian(obj, q)
            j = obj.robot.jacob0(q, 'trans');
        end
        
        function output = compute_S(obj, q)
            % Compute the joint limitation matrix S as described in the 
            % appendix of the JT-DS paper. The diagonal values are always
            % between 0 and 1, and (when correctly integrated into the
            % DS) ensure that the system will not collide with its
            % boundaries given a sufficiently small timestep.
            Qmin = zeros(size(q));
            Qmax = zeros(size(q));
            for i = 1:size(q,1)
                qlim = obj.robot.links(i).qlim;
                Qmin(i) = qlim(1); Qmax(i) = qlim(2);
            end
            s = 1- (2*(q - Qmin)./(Qmax - Qmin) - 1).^4;
            output = diag(s);
        end
    end
    
end

