% A script for generating a modified kuka robot (with 4 extra links).
% Real demonstrations are converted to demonstrations on the fake "nuka"
% robot by copying the 5th and 6th links to the 7th and 8th and 9th and
% 10th respectively, and the 7th to the 11th. These "new" link
% demonstrations are also given a small amount of noise.


close all
clear
clc
% 
% DH parameters for the KUKA LWR 4+ robot
dimq_kuka = 7;
A = [0 0 0 0 0 0 0.05];
Alpha = pi/2*[1 -1 -1 1 1 -1 0];
D = [.34 0 .4 0 .4 0 .279];
Qmin = 2*pi/180*[-85, -90, -100, -110, -140, -90, -120];
Qmax = 2*pi/180*[85, 90, 100, 110, 140, 90, 120];
% Create a model of the robot
kuka_robot = initialize_robot(A,D,Alpha,Qmin,Qmax);
% Now create a modified, "nuka" robot
dimq_nuka = 11;
A = [0 0 0 0 0 0 0 0 0 0 0.05];
Alpha = pi/2*[1 -1 -1 1 1 -1 -1 1 1 -1 0];
D = [.34 0 .4 0 .15 0 .15 0 .15 0 .279];
Qmin = -2*pi/180*ones(1, dimq_nuka);
Qmax = 2*pi/180*ones(1, dimq_nuka);
nuka_robot = initialize_robot(A,D,Alpha,Qmin,Qmax);
kuka_to_nuka_demonstration = @(x) [x(1:4, :); x(5:6, :) + 0.1*rand(size(x(5:6, :))); x(5:6, :) + 0.1*rand(size(x(5:6, :))); x(5:6, :) + 0.1*rand(size(x(5:6, :))); x(7, :)];

%fig = initialize_robot_figure(kuka_robot);
% Create a model plant for the robot's motor controller
kuka_robotplant = RobotPlant(kuka_robot, 'end_trans');
nuka_robotplant = RobotPlant(nuka_robot, 'end_trans');

demos_location = '~/Downloads/fore_hand/data.mat';
[Qs, Ts] = ImportDemonstrations(demos_location);
i = 1;
disp(size(Qs{i}));
Qs_nuka = kuka_to_nuka_demonstration(Qs{i});
[Data_kuka, ~] = preprocess_demos_jtds(kuka_robotplant, {Qs{i}}, {Ts{i}}, .1);
[Data_nuka, ~] = preprocess_demos_jtds(nuka_robotplant, {Qs_nuka}, {Ts{i}}, .1);
Q1_kuka = Data_kuka(1:dimq_kuka, :);
Q1_nuka = Data_nuka(1:dimq_nuka, :);
thinning_ratio = 50; % In the KUKA case, we get 500 datapoints per second, so we shrink the data density considerably
Q1_kuka = Q1_kuka(:, 1:thinning_ratio:end);
Q1_nuka = Q1_nuka(:, 1:thinning_ratio:end);
fig = initialize_robot_figure(kuka_robot);
PlaybackTrajectory(kuka_robotplant, Q1_kuka, .001, fig);
%fig = initialize_robot_figure(nuka_robot);
%PlaybackTrajectory(nuka_robotplant, Q1_nuka, .001, fig);