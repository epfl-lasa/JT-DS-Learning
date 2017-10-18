function [fig, h] = PlaybackTrajectory(robotplant, Q, T, old_fig, color)
% Visualizes a robot's trajectory, displaying a series of robot joint positions and their corresponding
% times. If clicked before completion, will restart the playback.

% Arguments:
% robotplant - a RobotPlant object specifying the kinematics of the robot being simulated
% Q - a dimq x n matrix of robot joint positions
% T - either a single value corresponding to the timestep, OR
%     a 1 x n matrix of times corresponding to each joint position
% old_fig - optional, a figure in which to plot the trajectory (if not
%     specified another is generated)
% color - optional, the color of the trace being plotted (r, g, b, y,
%     etc.)
% 
% Outputs:
% fig - the figure in which the simulation was displated
% h - the axes on which the trace was plotted.

    restart_flag = 0;
    complete_flag = 0;
    if nargin < 4 % if axes unspecified
        fig = initialize_robot_figure(robotplant.robot);
    else
        fig = figure(old_fig);
    end
    if nargin < 5 % if color unspecified
        color = 'r';
    end
    
    n = size(Q, 2);
    if size(T) == [1 1]
        dT = [repmat(T, 1, n-1) 0];
    else
        dT = [diff(T) 0];
    end
    h = [];
    %disable unwanted figure modes 
    zoom off
    rotate3d off
    pan off
    brush off
    datacursormode off

    set(fig,'WindowButtonDownFcn',@(h,e)restart_animation(h,e));
    set(fig,'WindowButtonUpFcn',[]);
    
    frame_ind = 1;
    while 1 
        if ~complete_flag
            q = Q(:, frame_ind);
            robotplant.robot.plot(transpose(q));
            x = robotplant.forward_kinematics(q);
            h_tmp = plot3(x(1), x(2), x(3), sprintf('.%s', color));
            h = [h h_tmp];
            robotplant.robot.delay = dT(frame_ind);
            frame_ind = frame_ind + 1;
            if frame_ind > n
                complete_flag = 1;
                break
            end
        end
        if restart_flag
            restart_flag = 0;
            frame_ind = 1;
            complete_flag = 0;
        end
    end
        
        
        function restart_animation(h,e)
            restart_flag = 1;
        end

end

