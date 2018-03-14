close all
clc
clear
load('JT_DS_comparison.mat')
load('SEDS_comparison.mat')

%%

% prepare data

data=vertcat(dist_P_JT_cell,dist_P_cell);

xlab={'Backward','Forward','Pouring','Obstacle 1',' Obstacle 2','Foot-Step','Singularity Motions'};
col=[102,255,255, 100;
0, 0, 255, 100];
col=col/255;
figure1 = figure;

axes1 = axes('Parent',figure1);
multiple_boxplot(data',xlab,{'JT-DS', 'SEDS+IK'},col')

xlabel('Scenario','Interpreter','latex');
title('Distance between the demonstrated end-effector position and the corresponding execution ',...
    'Interpreter','latex');
ylabel('$m^2$','Interpreter','latex');
box(axes1,'on');
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',22,'TickLabelInterpreter','latex')

%%

data=vertcat(dist_O_JT_cell,dist_O_cell);

xlab={'Backward','Forward','Pouring','Obstacle 1',' Obstacle 2','Foot-Step','Singularity Motions'};
col=[102,255,255, 100;
0, 0, 255, 100];
col=col/255;
figure1 = figure;

axes1 = axes('Parent',figure1);
multiple_boxplot(data',xlab,{'JT-DS', 'SEDS+IK'},col')

xlabel('Scenario','Interpreter','latex');
title('Distance between the demonstrated end-effector orientation and the corresponding execution ',...
    'Interpreter','latex');
box(axes1,'on');
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',22,'TickLabelInterpreter','latex')


%%

data=vertcat(dist_P_JT_E_JT_cell,dist_P_E_cell);

xlab={'Backward','Forward','Pouring','Obstacle 1',' Obstacle 2','Foot-Step','Singularity Motions'};
col=[102,255,255, 100;
0, 0, 255, 100];
col=col/255;
figure1 = figure;

axes1 = axes('Parent',figure1);
multiple_boxplot(data',xlab,{'JT-DS', 'SEDS+IK'},col')

xlabel('Scenario','Interpreter','latex');
ylabel('$m^2$','Interpreter','latex');
title('Distance between the desired target and the final end-effector positions ',...
    'Interpreter','latex');
box(axes1,'on');
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',22,'TickLabelInterpreter','latex')


%%

data=vertcat(dist_O_JT_E_JT_cell,dist_O_E_cell);

xlab={'Backward','Forward','Pouring','Obstacle 1',' Obstacle 2','Foot-Step','Singularity Motions'};
col=[102,255,255, 100;
0, 0, 255, 100];
col=col/255;
figure1 = figure;

axes1 = axes('Parent',figure1);
multiple_boxplot(data',xlab,{'JT-DS', 'SEDS+IK'},col')

xlabel('Scenario','Interpreter','latex');
ylabel('$m^2$','Interpreter','latex');
title('Distance between the desired target and the final end-effector orientation ',...
    'Interpreter','latex');
box(axes1,'on');
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',22,'TickLabelInterpreter','latex')


