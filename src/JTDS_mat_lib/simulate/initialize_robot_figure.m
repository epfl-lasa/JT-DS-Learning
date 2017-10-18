function fig = initialize_robot_figure(robot,varargin)
% Creates a simple visualization of the specified robot
% If the second argument is a figure, then plots the robot in that figure

if nargin>1
    % figure exists and we should activate axes to plot in
    axes(varargin{1})
else
    % create plot in new figure window
    fig = figure();
end
robot.plot(zeros(1, robot.n));
view([0 90]) % This positions the camera above the xy plane looking downward
hold on

end
