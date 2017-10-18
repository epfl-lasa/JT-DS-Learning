function [q, qd] = Simulation(q0, xt, motion_generator, dt,  fig, markertype)
%SIMULATION Simulates forward in real-time a DS policy in a given system

% Arguments
% q0 - dimq x 1, the initial joint-space configuration of the robot
% xt - dimx x 1, the ultimate task-space target for the robot
% motion_generator - a MotionGenerator/(child class) object for computing
% the simulatable policy
% dt - optional, provides the timestep to step forward between each frame
% fig - optional, the figure to plot the simulation in, if not specified a new one is
% generated
% markertype - input to plot like 'ro', 'bx', for plotting the task-space
% trace of the robot

% Output
% q - the joint-space positions achieved by the robot throughout the
% simulation
% qd - the joint-space velocities achieved by the robot throughout the
% simulation

    if nargin < 4 || isempty(dt) % If a timestep isn't specified, default is .01
        dt = .01;
    end
    goal_threshold = .05; % Determines how far from the goal the simulation should accept
    
    robot = motion_generator.plant.robot;
    if nargin < 5
        fig = initialize_robot_figure(robot);
    else
        figure(fig);
    end
    q = q0;
    robot.animate(q');
    if nargin < 6
        markertype = 'ro';
    end
    plot(xt(1), xt(2), markertype, 'markersize', 20);
    x = motion_generator.plant.forward_kinematics(q0);
    while(norm(x - xt) > goal_threshold) % at each time step, system is integrated using the Newton metho
        qd = motion_generator.get_next_motion(q, xt, dt);
        q_candidate = q + qd*dt;
        q = q_candidate;
        x = motion_generator.plant.forward_kinematics(q);
        robot.plot(q');
        if ~ishandle(fig)
            error('Figure closed prematurely');
        end
        plot(x(1), x(2), 'b.', 'markersize', 10);
        pause(.2);
    end
    disp('Reached goal.');
end

