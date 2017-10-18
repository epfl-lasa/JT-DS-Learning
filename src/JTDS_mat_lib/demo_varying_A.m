% This file allows you to observe differing behaviors of the robot motion
% controller based on different values of the joint augmentation matrix A.

% First, we define the parameters of the system:
dimq = 3;
% The following are the DH parameters for our 3 DOF robot
A = [.4, .4, .1];
D = [0, 0,0]; % All components are in the 2D plane
Alpha = [0, 0,0]; 
Qmin = -pi*[1, 1, -.25]; % No joint limits except for on the ankle
Qmax = pi*[1, 1, .75];
% Create a model of the robot
robot = initialize_robot(A,D,Alpha,Qmin,Qmax);
fig = initialize_robot_figure(robot);             
% Create a model plant for the robot's motor controller
robotplant = RobotPlant(robot, 'end_trans');


% We write out a number of different A matrices (each emphasizing a
% different joint) and build controllers to match them.
q_init = [-pi/2;-pi/6;0]; x_targets = [0.5; -.3; 0];
identity_mapping = compute_mapping(eye(dimq), 'None');
mg_vanilla = MotionGeneratorBounded(robotplant, zeros(dimq, 1), eye(dimq), 1, eye(dimq), identity_mapping);
A_hip = [5,0,0; 0, 1, 0; 0,0, 1];
mg_hip = MotionGeneratorBounded(robotplant, zeros(dimq, 1), eye(dimq), 1, A_hip, identity_mapping);
A_knee = [1,0,0; 0, 5, 0; 0,0, 1];
mg_knee = MotionGeneratorBounded(robotplant, zeros(dimq, 1), eye(dimq), 1, A_knee, identity_mapping);
A_wrist = [1,0,0; 0, 1, 0; 0,0, 5];
mg_wrist = MotionGeneratorBounded(robotplant, zeros(dimq, 1), eye(dimq), 1, A_wrist, identity_mapping);

% We interpolate the resulting trajectories forward
[Q_vanilla, T_vanilla]...
 = computeFullTrajectory(q_init, x_targets, mg_vanilla);
[Q_hip, T_hip] = computeFullTrajectory(q_init, x_targets, mg_hip);
[Q_knee, T_knee] = computeFullTrajectory(q_init, x_targets, mg_knee);
[Q_wrist, T_wrist] = computeFullTrajectory(q_init, x_targets, mg_wrist);

% and visualize them overlayed on each other
if ~ishandle(fig)
    fig = initialize_robot_figure(robot);
else
    figure(fig);
end
plot3(x_targets(1,:),x_targets(2,:),x_targets(3,:), 'ro', 'markersize', 20);
[~, h_wrist] = PlaybackTrajectory(robotplant, Q_wrist, T_wrist, fig, 'g');
[~, h_hip] =PlaybackTrajectory(robotplant, Q_hip, T_hip, fig, 'r');
[~, h_knee] =PlaybackTrajectory(robotplant, Q_knee, T_knee, fig, 'b');
robot.plot(q_init');
legend([h_hip(1), h_knee(1), h_wrist(1)], {'Motion A', 'Motion B', 'Motion C'});

%% Finally, we plot the different joint submotions over time
Qd_hip = [diff(Q_hip, 1, 2)./repmat(diff(T_hip), dimq, 1) zeros(dimq, 1)];
Qd_knee = [diff(Q_knee, 1, 2)./repmat(diff(T_knee), dimq, 1) zeros(dimq, 1)];
Qd_wrist = [diff(Q_wrist, 1, 2)./repmat(diff(T_wrist), dimq, 1) zeros(dimq, 1)];

figure()
hold on
color = ['r', 'g', 'b'];
name = {'Hip', 'Knee', 'Wrist'};
for i = 1:dimq
    ax1 = subplot(3,1,i);
    hold on
    plot(linspace(0, 1, size(Q_hip,2)-20), Q_hip(i,21:end), 'r','LineWidth', 2.5);
    plot(linspace(0, 1, size(Q_knee,2)-20), Q_knee(i,21:end), 'g','LineWidth', 2.5);
    plot(linspace(0, 1, size(Q_wrist,2)-20), Q_wrist(i,21:end), 'b','LineWidth', 2.5);
    title(sprintf('%s Positions', name{i}));
end
ax1.YLim = [-3, 3];
linkaxes([ax1, ax2, ax3], 'y')
l = legend('Motion A', 'Motion B','Motion C')
set(l, 'FontSize', 14)
hold off;

% Note that when a particular joint has a large A component, that joint
% tends to dominate the motion as shown in the generated graphs.