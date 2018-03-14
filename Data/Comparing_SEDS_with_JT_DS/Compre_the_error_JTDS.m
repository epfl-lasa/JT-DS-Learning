clc
close all
clear
List={'back','fore','pour','pour_obst','pour_obst_2','foot','singularity'};


for ii=1:7
    choosen_dataset=List{ii};
    clearvars -except dist_P_JT dist_O_JT LABEL_JT Data_E_O_JT Data_E_P_JT Data_T_O_JT Data_T_P_JT choosen_dataset ii List dist_P_JT_E_JT dist_O_JT_E_JT
    switch choosen_dataset
        case 'back'
            Name='Backward Reaching';
            load('JT_DS/back/model.mat')
            Data_E{1}=importfile_JTDS('JT_DS/back/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_JTDS('JT_DS/back/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_JTDS('JT_DS/back/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_JTDS('JT_DS/back/TheRobotTrajectory4.txt');
            Data_E{5}=importfile_JTDS('JT_DS/back/TheRobotTrajectory5.txt');
        case 'fore'
            Name='Forward Reaching';
            load('JT_DS/fore_hand/model.mat')
            Data_E{1}=importfile_JTDS('JT_DS/fore_hand/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_JTDS('JT_DS/fore_hand/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_JTDS('JT_DS/fore_hand/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_JTDS('JT_DS/fore_hand/TheRobotTrajectory4.txt');
            Data_E{5}=importfile_JTDS('JT_DS/fore_hand/TheRobotTrajectory5.txt');
            Data_E{6}=importfile_JTDS('JT_DS/fore_hand/TheRobotTrajectory6.txt');
        case 'pour'
            Name='Pouring';
            load('JT_DS/pour/model.mat')
            Data_E{1}=importfile_JTDS('JT_DS/pour/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_JTDS('JT_DS/pour/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_JTDS('JT_DS/pour/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_JTDS('JT_DS/pour/TheRobotTrajectory4.txt');
            Data_E{5}=importfile_JTDS('JT_DS/pour/TheRobotTrajectory5.txt');
        case 'pour_obst'
            Name='Pouring - Obstacle 1';
            load('JT_DS/Pour_obst/model.mat')
            Data_E{1}=importfile_JTDS('JT_DS/Pour_obst/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_JTDS('JT_DS/Pour_obst/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_JTDS('JT_DS/Pour_obst/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_JTDS('JT_DS/Pour_obst/TheRobotTrajectory4.txt');
            Data_E{5}=importfile_JTDS('JT_DS/Pour_obst/TheRobotTrajectory5.txt');
            Data_E{6}=importfile_JTDS('JT_DS/Pour_obst/TheRobotTrajectory6.txt');
        case 'pour_obst_2'
            Name='Pouring - Obstacle 2';
            load('JT_DS/pour_obst_2/model.mat')
            Data_E{1}=importfile_JTDS('JT_DS/pour_obst_2/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_JTDS('JT_DS/pour_obst_2/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_JTDS('JT_DS/pour_obst_2/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_JTDS('JT_DS/pour_obst_2/TheRobotTrajectory4.txt');
        case 'foot'
            Name='Foot Step';
            load('JT_DS/Foot/model.mat')
            Data_E{1}=importfile_JTDS('JT_DS/Foot/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_JTDS('JT_DS/Foot/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_JTDS('JT_DS/Foot/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_JTDS('JT_DS/Foot/TheRobotTrajectory4.txt');
            Data_E{5}=importfile_JTDS('JT_DS/Foot/TheRobotTrajectory5.txt');
        case 'singularity'
            Name='Singularity Motions';
            load('JT_DS/singularity/model.mat')
            Data_E{1}=importfile_JTDS('JT_DS/singularity/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_JTDS('JT_DS/singularity/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_JTDS('JT_DS/singularity/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_JTDS('JT_DS/singularity/TheRobotTrajectory4.txt');
            Data_E{5}=importfile_JTDS('JT_DS/singularity/TheRobotTrajectory5.txt');
            Data_E{6}=importfile_JTDS('JT_DS/singularity/TheRobotTrajectory6.txt');
    end
    
    
    Data_dummpy=Data_train;
    Data_train=[];
    for i=1:(size(index_train,2)-1)
        Data_train_joints{i}=Data_dummpy(1:7,index_train(i):index_train(i+1)-1)';
    end
    for i=1:(size(index_train,2)-1)
        Data_T_O_JT{i,ii}=zeros(size(Data_train_joints{i},1),6);
        Data_T_P_JT{i,ii}=zeros(size(Data_train_joints{i},1),3);
        for j=1:size(Data_train_joints{i},1)
            tmp=robotplant.end_pos_orien(Data_train_joints{i}(j,:));
            Data_T_O_JT{i,ii}(j,:)=tmp(1:6)';
            Data_T_P_JT{i,ii}(j,:)=tmp(7:9)';
        end
        Data_E_P_JT{i,ii}=Data_E{i}(:,7:9);
        Data_E_O_JT{i,ii}=Data_E{i}(:,1:6);
    end
    distance=0;
    for i=1:(size(index_train,2)-1)
        distance=distance+norm(Data_E_P_JT{i,ii}(1,:)-Data_T_P_JT{i,ii}(1,:))+norm(Data_E_O_JT{i,ii}(1,:)-Data_T_O_JT{i,ii}(1,:));
    end
    if distance>0.2
        keyboard
    end
    
    for i=1:(size(index_train,2)-1)
        dist_P_JT_E_JT(i,ii)=norm(Data_E_P_JT{i,ii}(end,:)-Data_E{i}(1,16:18));
        dist_O_JT_E_JT(i,ii)=norm(Data_E_O_JT{i,ii}(end,:)-Data_E{i}(1,10:15));
        [~,ix,iy] =dtw(Data_E_P_JT{i,ii}',Data_T_P_JT{i,ii}','squared');
        Data_E_P_JT_dummy=Data_E_P_JT{i,ii}(ix,:);
        Data_T_P_JT_dummy=Data_T_P_JT{i,ii}(iy,:);
        dist_P_JT(i,ii)=norm(Data_E_P_JT_dummy-Data_T_P_JT_dummy,2);
        
        Data_E_O_JT_dummy=Data_E_O_JT{i,ii}(ix,:);
        Data_T_O_JT_dummy=Data_T_O_JT{i,ii}(iy,:);
        dist_O_JT(i,ii)=norm(Data_E_O_JT_dummy-Data_T_O_JT_dummy,2);
        LABEL_JT{i,ii}=Name;
    end
    
end
%%

tmp=reshape(dist_P_JT,42,1);
Position_error_JT=tmp(tmp~=0);
for i=1:7
    dist_P_JT_cell{i}=dist_P_JT(dist_P_JT(:,i)~=0,i);
end

tmp=reshape(dist_O_JT,42,1);
Orientation_error_JT=tmp(tmp~=0);
for i=1:7
    dist_O_JT_cell{i}=dist_O_JT(dist_O_JT(:,i)~=0,i);
end

tmp=reshape(dist_P_JT_E_JT,42,1);
Position_error_End_JT=tmp(tmp~=0);
for i=1:7
    dist_P_JT_E_JT_cell{i}=dist_P_JT_E_JT(dist_P_JT_E_JT(:,i)~=0,i);
end

tmp=reshape(dist_O_JT_E_JT,42,1);
Orientation_error_End_JT=tmp(tmp~=0);
for i=1:7
    dist_O_JT_E_JT_cell{i}=dist_O_JT_E_JT(dist_O_JT_E_JT(:,i)~=0,i);
end

LABEL_JT=reshape(LABEL_JT,42,1);
LABEL_JT=LABEL_JT(tmp~=0);

%%
close all
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
boxplot(Position_error_JT,LABEL_JT)
xlabel('Scenario','Interpreter','latex');
title('Distance between the demonstrated end-effector position and the corresponding execution ',...
    'Interpreter','latex');
ylabel('$m^2$','Interpreter','latex');
box(axes1,'on');
grid(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',22,'TickLabelInterpreter','latex','XTick',...
    [1 2 3 4 5 6 7],'XTickLabel',...
    {'Backward','Forward','Pouring','Obstacle 1',' Obstacle 2','Foot-Step','Singularity Motions'});

figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
boxplot(Orientation_error_JT,LABEL_JT)
title('Distance between the demonstrated end-effector orientation and the corresponding execution ','Interpreter','latex')
xlabel('Scenario','Interpreter','latex');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'FontSize',22,'TickLabelInterpreter','latex','XTick',...
    [1 2 3 4 5 6 7],'XTickLabel',...
    {'Backward','Forward','Pouring','Obstacle 1',' Obstacle 2','Foot-Step','Singularity Motions'});

figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
boxplot(Position_error_End_JT,LABEL_JT)
title('Distance between the desired target and the final end-effector positions ','Interpreter','latex')
xlabel('Scenario','Interpreter','latex');
ylabel('$m^2$','Interpreter','latex');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'FontSize',22,'TickLabelInterpreter','latex','XTick',...
    [1 2 3 4 5 6 7],'XTickLabel',...
    {'Backward','Forward','Pouring','Obstacle 1',' Obstacle 2','Foot-Step','Singularity Motions'});

figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
boxplot(Orientation_error_End_JT,LABEL_JT)
title('Distance between the desired target and the final end-effector orientation ','Interpreter','latex')
xlabel('Scenario','Interpreter','latex');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'FontSize',22,'TickLabelInterpreter','latex','XTick',...
    [1 2 3 4 5 6 7],'XTickLabel',...
    {'Backward','Forward','Pouring','Obstacle 1',' Obstacle 2','Foot-Step','Singularity Motions'});