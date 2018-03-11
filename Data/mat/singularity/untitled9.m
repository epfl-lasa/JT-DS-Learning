clc
clear
close all

figure1 = figure;
subplot1 = subplot(1,8,[1 2 3 4 5],'Parent',figure1);
% Create axes
% axes1 = axes('Parent',subplot1);
hold(subplot1,'on');
xlabel(' X [m]','Interpreter','latex');

% Create zlabel
zlabel('Z [m]','Interpreter','latex');

% Create ylabel
ylabel('Y [m]','Interpreter','latex');

view(subplot1,[-37.5 30]);
grid(subplot1,'on');
% Set the remaining axes properties
set(subplot1,'FontSize',24,'TickLabelInterpreter','latex');
data = [];
for j=1:10
    Data=importfile(sprintf('TheRobotTrajectory%d.txt',j));
    data{j} = [Data(:,4),Data(:,5),Data(:,6),Data(:,7),Data(:,8),Data(:,9),Data(:,10)]';
    DET=zeros(1,size(Data,1));
    for i=1:size(Data,1)
        Jacobian=[Data(i,15),Data(i,16),Data(i,17);
            Data(i,18),Data(i,19),Data(i,20);
            Data(i,21),Data(i,22),Data(i,23)];
        DET(i)=det(Jacobian);
    end
    demo{j}=[Data(1:10:end,1),Data(1:10:end,2),Data(1:10:end,3)]';
    A=sprintf('$q^{%d}=%d^{\\circ}$',j,120-10*j);
    h3=plot3(Data(1,1),Data(1,2),Data(1,3),...
    'MarkerFaceColor',[0.0705882385373116 0.211764708161354 0.141176477074623],...
    'MarkerEdgeColor','none',...
    'MarkerSize',16,...
    'Marker','pentagram',...
    'LineStyle','none');
    h4=plot3(Data(end,1),Data(end,2),Data(end,3),...
    'MarkerFaceColor',[0.749019622802734 0 0.749019622802734],...
    'MarkerEdgeColor','none',...
    'MarkerSize',16,...
    'Marker','hexagram',...
    'LineStyle','none');

   h1= plot3(Data(:,1),Data(:,2),Data(:,3),'LineWidth',3,'LineStyle','--',...
    'Color',[0 0.447058826684952 0.74117648601532]);
   h2= plot3(Data(:,1),Data(:,2),Data(:,3),'LineWidth',1,...
    'Color',[00.850980401039124 0.325490206480026 0.0980392172932625]);
hold on
    max(DET)
end
for j=1:10
    Data=importfile(sprintf('SEDS/TheRobotTrajectory%d.txt',j));
   h5= plot3(Data(:,1),Data(:,2),Data(:,3),'LineWidth',3,...
    'Color',[0 0 0]);
hold on
end

axis equal

subplo2= subplot(1,8,[6 7 8],'Parent',figure1);

hold(subplo2,'on');
xlabel(' X [m]','Interpreter','latex');

% Create zlabel
ylabel('Z [m]','Interpreter','latex');


grid(subplo2,'on');
% Set the remaining axes properties
set(subplo2,'FontSize',24,'TickLabelInterpreter','latex');

for j=1:10
    Data=importfile(sprintf('TheRobotTrajectory%d.txt',j));
    h3=plot(Data(1,1),Data(1,3),...
    'MarkerFaceColor',[0.0705882385373116 0.211764708161354 0.141176477074623],...
    'MarkerEdgeColor','none',...
    'MarkerSize',16,...
    'Marker','pentagram',...
    'LineStyle','none');
    h4=plot(Data(end,1),Data(end,3),...
    'MarkerFaceColor',[0.749019622802734 0 0.749019622802734],...
    'MarkerEdgeColor','none',...
    'MarkerSize',16,...
    'Marker','hexagram',...
    'LineStyle','none');

   h1= plot(Data(:,1),Data(:,3),'LineWidth',3,'LineStyle','--',...
    'Color',[0 0.447058826684952 0.74117648601532]);
   h2= plot(Data(:,1),Data(:,3),'LineWidth',1,...
    'Color',[00.850980401039124 0.325490206480026 0.0980392172932625]);
hold on
    max(DET)
end
for j=1:10
    Data=importfile(sprintf('SEDS/TheRobotTrajectory%d.txt',j));
   h5= plot(Data(:,1),Data(:,3),'LineWidth',3,...
    'Color',[0 0 0]);
hold on
end

legend([h1,h2,h3,h4,h5],'The excuted motion', ...
                           'The demostrated motion','The initial positions','The target positions','Cartesian motion generator')
legend1 = legend(subplo2,'show');
set(subplo2,'Interpreter','latex');
% figure1 = figure;
% 
% % Create axes
% axes1 = axes('Parent',figure1);
% hold(axes1,'on');
% xlabel(' Time [s]','Interpreter','latex');
% 
% % Create ylabel
% ylabel('Determinant of Jacobian','Interpreter','latex');
% 
% grid(axes1,'on');
% % Set the remaining axes properties
% set(axes1,'FontSize',24,'TickLabelInterpreter','latex');
% 
% for j=1:12
%     Data=importfile(sprintf('TheRobotTrajectory%d.txt',j));
%     DET=zeros(1,size(Data,1));
%     for i=1:size(Data,1)
%         Jacobian=[Data(i,15),Data(i,16),Data(i,17);
%             Data(i,18),Data(i,19),Data(i,20);
%             Data(i,21),Data(i,22),Data(i,23)];
%         DET(i)=det(Jacobian);
%     end
%     DET=smooth(DET,'loess');
%     A=0:0.001:0.001*(size(Data,1)-1);
%     plot(A,DET,'LineWidth',1,'LineStyle','-',...
%     'Color',[0 0 0])
% hold on
%     max(DET)
% end