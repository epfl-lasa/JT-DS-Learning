function robot_copy_names = GraphRobot(robot, Q, fig, robot_plot_options )
% Overlays the robot in the given joint configuration on the figure
% Arguments:
% robot - a Corke robotics-toolkit style robot
% Q - a dimq x n matrix of robot joint positions to plot, each with a
% separate robot
% fig - if specified, plot the robot in this figure
% robot_plot
% robot_plot_options - a Corke robotics-toolbox plotopt structure to
% specify the plot type
%
% Output:
% robot_copy_names - the names of each of the generated robots plotted
    robot_copy_names = [];
    if nargin < 3
        fig = [];
    end
    for i = 1:size(Q, 2)
        q = Q(:, i);
        new_robot = SerialLink(robot);
        new_robot.name = sprintf('robot copy %d', rand*10000);
        robot_copy_names = [robot_copy_names new_robot.name];
        if nargin < 4 || isempty(robot_plot_options)
            new_robot.plotopt = {'jointdiam', 3,'basewidth', .5, 'noshadow', 'notiles', 'noshading','noname', 'nowrist', 'linkcolor', (0.5 + 0.3*i/size(Q,2))*[1,1,1], 'jointcolor', (0.4 + 0.3*i/size(Q,2))*[1,1,1]};
        else
            new_robot.plotopt = robot_plot_options;
        end
        if isempty(fig)
            fig = initialize_robot_figure(new_robot);
        end
        figure(fig);
        new_robot.plot(q');
    end
end

