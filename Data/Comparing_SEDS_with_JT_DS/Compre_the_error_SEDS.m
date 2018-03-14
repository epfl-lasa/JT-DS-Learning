clc
close all
clear
List={'back','fore','pour','pour_obst','pour_obst_2','foot','singularity'};


for ii=1:7
    choosen_dataset=List{ii};
    clearvars -except dist_P dist_O LABEL Data_E_O Data_E_P Data_T_O Data_T_P choosen_dataset ii List dist_P_E dist_O_E
    switch choosen_dataset
        case 'back'
            Name='Backward Reaching';
            load('SEDS/back/model.mat')
            Data_E{1}=importfile_SEDS('SEDS/back/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_SEDS('SEDS/back/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_SEDS('SEDS/back/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_SEDS('SEDS/back/TheRobotTrajectory4.txt');
            Data_E{5}=importfile_SEDS('SEDS/back/TheRobotTrajectory5.txt');
        case 'fore'
            Name='Forward Reaching';
            load('SEDS/fore_hand/model.mat')
            Data_E{1}=importfile_SEDS('SEDS/fore_hand/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_SEDS('SEDS/fore_hand/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_SEDS('SEDS/fore_hand/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_SEDS('SEDS/fore_hand/TheRobotTrajectory4.txt');
            Data_E{5}=importfile_SEDS('SEDS/fore_hand/TheRobotTrajectory5.txt');
            Data_E{6}=importfile_SEDS('SEDS/fore_hand/TheRobotTrajectory6.txt');
        case 'pour'
            Name='Pouring';
            load('SEDS/pour/model.mat')
            Data_E{1}=importfile_SEDS('SEDS/pour/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_SEDS('SEDS/pour/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_SEDS('SEDS/pour/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_SEDS('SEDS/pour/TheRobotTrajectory4.txt');
        case 'pour_obst'
            Name='Pouring - Obstacle 1';
            load('SEDS/Pour_obst/model.mat')
            Data_E{1}=importfile_SEDS('SEDS/Pour_obst/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_SEDS('SEDS/Pour_obst/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_SEDS('SEDS/Pour_obst/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_SEDS('SEDS/Pour_obst/TheRobotTrajectory4.txt');
            Data_E{5}=importfile_SEDS('SEDS/Pour_obst/TheRobotTrajectory5.txt');
        case 'pour_obst_2'
            Name='Pouring - Obstacle 2';
            load('SEDS/pour_obst_2/model.mat')
            Data_E{1}=importfile_SEDS('SEDS/pour_obst_2/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_SEDS('SEDS/pour_obst_2/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_SEDS('SEDS/pour_obst_2/TheRobotTrajectory3.txt');
        case 'foot'
            Name='Foot Step';
            load('SEDS/foot/model.mat')
            Data_E{1}=importfile_SEDS('SEDS/foot/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_SEDS('SEDS/foot/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_SEDS('SEDS/foot/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_SEDS('SEDS/foot/TheRobotTrajectory4.txt');
        case 'singularity'
            Name='Singularity Motions';
            load('SEDS/singularity/model.mat')
            Data_E{1}=importfile_SEDS('SEDS/singularity/TheRobotTrajectory1.txt');
            Data_E{2}=importfile_SEDS('SEDS/singularity/TheRobotTrajectory2.txt');
            Data_E{3}=importfile_SEDS('SEDS/singularity/TheRobotTrajectory3.txt');
            Data_E{4}=importfile_SEDS('SEDS/singularity/TheRobotTrajectory4.txt');
            Data_E{5}=importfile_SEDS('SEDS/singularity/TheRobotTrajectory5.txt');
    end
    
    
    Data_dummpy=Data;
    Data=[];
    for i=1:(size(index,2)-1)
        Target{i}=Data_E{i}(1,7:12);
        Data{i}=Data_dummpy(1:6,index(i):index(i+1)-1)+repmat(Target{i}',1,size(Data_dummpy(1:6,index(i):index(i+1)-1),2));
    end
    for i=1:(size(index,2)-1)
        Data_T_P{i,ii}=Data{i}(4:6,:)';
        Data_T_O{i,ii}=zeros(size(Data{i}(1:3,:)',1),6);
        for j=1:size(Data{i}(1:3,:)',1)
            tmp=axang2rotm([Data{i}(1:3,j)/norm(Data{i}(1:3,j)) ;norm(Data{i}(1:3,j))]');
            Data_T_O{i,ii}(j,:)=[tmp(:,1) ;tmp(:,2)]';
        end
        Data_E_P{i,ii}=Data_E{i}(:,4:6);
        
        Data_E_O{i,ii}=zeros(size(Data_E{i}(:,1:3)',1),6);
        for j=1:size(Data_E{i}(:,1:3),1)
            tmp=axang2rotm([Data_E{i}(j,1:3)/norm(Data_E{i}(j,1:3)) norm(Data_E{i}(j,1:3))]);
            Data_E_O{i,ii}(j,:)=[tmp(:,1) ;tmp(:,2)]';
        end
    end
    distance=0;
    for i=1:(size(index,2)-1)
        distance=distance+norm(Data_E_P{i,ii}(1,:)-Data_T_P{i,ii}(1,:))+norm(Data_E_O{i,ii}(1,:)-Data_T_O{i,ii}(1,:));
    end
    if distance>0.3
        keyboard
    end
    
    for i=1:(size(index,2)-1)
        dist_P_E(i,ii)=norm(Data_E_P{i,ii}(end,:)-Target{i}(4:6));
        tmp=axang2rotm([Target{i}(1:3)/norm(Target{i}(1:3)) norm(Target{i}(1:3))]);
        dist_O_E(i,ii)=norm(Data_E_O{i,ii}(end,:)-[tmp(:,1) ;tmp(:,2)]');
        [~,ix,iy] =dtw(Data_E_P{i,ii}',Data_T_P{i,ii}','squared');
        Data_E_P_dummy=Data_E_P{i,ii}(ix,:);
        Data_T_P_dummy=Data_T_P{i,ii}(iy,:);
        dist_P(i,ii)=norm(Data_E_P_dummy-Data_T_P_dummy,2);
        
        Data_E_O_dummy=Data_E_O{i,ii}(ix,:);
        Data_T_O_dummy=Data_T_O{i,ii}(iy,:);
        dist_O(i,ii)=norm(Data_E_O_dummy-Data_T_O_dummy,2);
        LABEL{i,ii}=Name;
    end
    
end
%%

tmp=reshape(dist_P,42,1);
Position_error=tmp(tmp~=0);
for i=1:7
    dist_P_cell{i}=dist_P(dist_P(:,i)~=0,i);
end

tmp=reshape(dist_O,42,1);
Orientation_error=tmp(tmp~=0);
for i=1:7
    dist_O_cell{i}=dist_O(dist_O(:,i)~=0,i);
end

tmp=reshape(dist_P_E,42,1);
Position_error_End=tmp(tmp~=0);
for i=1:7
    dist_P_E_cell{i}=dist_P_E(dist_P_E(:,i)~=0,i);
end

tmp=reshape(dist_O_E,42,1);
Orientation_error_End=tmp(tmp~=0);
for i=1:7
    dist_O_E_cell{i}=dist_O_E(dist_O_E(:,i)~=0,i);
end

LABEL=reshape(LABEL,42,1);
LABEL=LABEL(tmp~=0);

%%
close all
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
boxplot(Position_error,LABEL)
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
boxplot(Orientation_error,LABEL)
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
boxplot(Position_error_End,LABEL)
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
boxplot(Orientation_error_End,LABEL)
title('Distance between the desired target and the final end-effector orientation ','Interpreter','latex')
xlabel('Scenario','Interpreter','latex');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'FontSize',22,'TickLabelInterpreter','latex','XTick',...
    [1 2 3 4 5 6 7],'XTickLabel',...
    {'Backward','Forward','Pouring','Obstacle 1',' Obstacle 2','Foot-Step','Singularity Motions'});