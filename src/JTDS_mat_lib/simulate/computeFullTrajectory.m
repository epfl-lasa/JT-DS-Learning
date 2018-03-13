function [Q, T] = computeFullTrajectory(q_initial, x_targets, motion_generator, goal_tolerance, max_duration,orientation_flag)
%COMPUTEFULLTRAJECTORY Interpolates a JT-DS trajectory
%   This function integrates a JT-DS problem and computes the evolution of
%   the system over a given timespan, using an ODE solver. Each target is
%   integrated for up to a fixed timespan, or until the target reaches the
%   goal.

%   Arguments:
%   q_initial - a dimq x 1 vector denoting the initial joint configuration
%   x_targets - a dimx x #targets matrix denoting the desired task-space
%   positions for the system to move to
%   motion_generator - a MotionGenerator/(child class) obj obj to compute the DS motion
%   goal_tolerance - the distance from the target position necessary for
%   the controller to "accept"
%   max_duration - the maximum period of time a trajectory will be
%   interpolated. After this period elapses, the controller begins
%   integrating the motion to the next target on the list.
%   Outputs:
%   Q is an n x dimq matrix denoting the interpolated robot positions
%   T is an n x 1 matrix denoting the times corresponding to each position
    if nargin < 4
        goal_tolerance = 0.03;
    end
    if nargin < 5
        max_duration = 60; %seconds
    end
    
    T = []; Q = [];
    q_start = q_initial;
    for i = 1:size(x_targets, 2)
        x_target = x_targets(:, i);
        [T_subtraj, Q_subtraj] = ode15s(motion_generator.ODE_fun(x_target, 1,orientation_flag), ...
            [0, max_duration], q_start, ...
            motion_generator.ODE_options(x_target, goal_tolerance,orientation_flag));
        if isempty(T)
            T = T_subtraj;
        else
            T = [T; T_subtraj];
        end
        if sum(sum(isnan(Q_subtraj)))
            error('Generated a NaN... something is wrong with ODE solution. Consider switching between ode15s and ode45 in computeFullTrajectory.')
        end
        Q = [Q; Q_subtraj];
        q_start = Q_subtraj(end, :)';
    end
    T = T';
    Q = Q';

end

