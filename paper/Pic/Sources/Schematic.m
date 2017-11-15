clc
close all
clear
startup_rvc


l1=1.5;
l2=1.5;

q1_limit=[-3.14:0.01:3.14];
q2_limit=[-3.14:0.01:3.14];
x_limit=[0:0.01:4];
y_limit=[0:0.01:4];
X_d=[2;2];




for i=1:size(y_limit,2)
    H=[x_limit;repmat(y_limit(i),1,size(y_limit,2))];
    V(i,:)=sum((H-repmat(X_d,1,size(H,2))).*(H-repmat(X_d,1,size(H,2))),1);
end

Xlim=repmat(x_limit,size(y_limit,2),1);
Ylim=repmat(y_limit',1,size(x_limit,2));

subplot1 = subplot(1,2,1);
hold(subplot1,'on');

L(1) = Link([0 0 l1 0],'standard')
L(2)= Link([0 0 l2 0],'standard')
L(3)= Link([0 0 0 0],'standard')
r = SerialLink(L,'name','two link')
r.plotopt = {'noshadow','nojaxes', 'nowrist','noname','linkcolor',0.7*[1,1,1], 'ortho','noshading','notiles','jointcolor',0.4*[1,1,1]};
r.plot([pi/3,-pi/4,0])
hold on
contourf(Xlim,Ylim,V)
colormap(cool)
axis equal
xlabel('$X~ [m]$','Interpreter','latex');

% Create ylabel
ylabel('$Y~ [m]$','Interpreter','latex');
box(subplot1,'on');
axis(subplot1,'tight');
set(subplot1,'BoxStyle','full','FontSize',24,'Layer','top',...
    'TickLabelInterpreter','latex');
xlim(subplot1,[x_limit(1) x_limit(end)]);
ylim(subplot1,[y_limit(1) y_limit(end)]);

q1lim=repmat(q1_limit,size(q1_limit,2),1);
q2lim=repmat(q2_limit',1,size(q1_limit,2));


for i=1:size(q1_limit,2)
    for j=1:size(q2_limit,2)
    q1lim(i,j)= q1_limit(i);
    q2lim(i,j)= q2_limit(j);
    H=[l1*cos(q1_limit(i))+l2*cos(q2_limit(j)+q1_limit(i));l1*sin(q1_limit(i))+l2*sin(q2_limit(j)+q1_limit(i))];
    V_q(i,j)=sum((H-X_d).*(H-X_d),1);
    end
end
subplot1 = subplot(1,2,2);
hold(subplot1,'on');
contourf(q1lim,q2lim,V_q)
xlabel('$q_1~ [rad]$','Interpreter','latex');

% Create ylabel
ylabel('$q_2~ [rad]$','Interpreter','latex');
box(subplot1,'on');
colormap(cool)
axis(subplot1,'tight');
set(subplot1,'BoxStyle','full','FontSize',24,'Layer','top',...
    'TickLabelInterpreter','latex');
axis equal