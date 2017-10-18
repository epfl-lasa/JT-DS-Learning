function [Data, index] = preprocess_demos_jtds(robotplant, demos, time, tol_cutting)
% This function preprocesses raw joint demonstration data and molds it into
% a suitable format.
% The function computes the first time derivative of demonstrations and
% trims the data. The function can be
% called using: 
%
%          [Data, index] = preprocess_demos(demos,time,tol_cutting)
%
% Inputs -----------------------------------------------------------------
%
%
%   o demos:   A variable containing all demonstrations (only
%              joint trajectories). The variable 'demos' should follow the
%              following format:
%              - demos{n}: qdim x T^n matrix representing the qdim dimensional
%                          joint trajectories. T^n is the number of datapoints in
%                          each demonstration (1 < n < N)
%
%   o time:    This variable can be provided in two ways. If time steps
%              between all demonstration points are the same, 'time' could be
%              given as a positive scalar (i.e. time = dt). If not, 'time'
%              should follow the following format:
%              - time{n}: 1 x T^n vector representing the time array of length
%                         T^n corresponding to each demo  (1 < n < N)
%
%   o tol_cutting:  A small positive scalar that is used to trim data. It
%                   removes the redundant datapoint from the begining and
%                   the end of each demonstration that their first time
%                   derivative is less than 'tol_cutting'. This is not
%                   strictly necessary for JT-DS; however from practical point of
%                   view, it is very useful. There are always lots of noisy
%                   data at the begining (before the user starts the
%                   demonstration) and the end (after the user finished the
%                   demonstration) of each demosntration that are not
%                   useful.
%
% Outputs ----------------------------------------------------------------
%
%   o xT:      qdim x N array representing the each of the demonstration's
%              final points (target points).
%
%   o Data:    A (2*qdim + xdim) x N_Total matrix containing all demonstration data points.
%              The rows are layed out as follows:
%              1:qdim = joint angles
%              qdim + 1: 2*qdim = joint velocities
%              2*qdim + 1:2*qdim + xdim = target position
%
%              Each column of Data stands
%              for a datapoint. All demonstrations are put next to each other 
%              along the second dimension. For example, if we have 3 demos
%              D1, D2, and D3, then the matrix Data is:
%                               Data = [[D1] [D2] [D3]]
%
%   o index:   A vector of N+1 components defining the initial index of each
%              demonstration. For example, index = [1 T1 T2 T3] indicates
%              that columns 1:T1-1 belongs to the first demonstration,
%              T1:T2-1 -> 2nd demonstration, and T2:T3-1 -> 3rd
%              demonstration.
    
%checking if a fixed time step is provided or not.
if length(time)==1
    dt = time;
end

dimq = size(demos{1},1); %dimensionality of demonstrations
dimx = robotplant.dimx; %dimensionality of the task space
Data=[];
index = 1;

for i=1:length(demos)
    clear tmp_full tmpq tmpq_d tmpx tmpxd
    
    % de-noising data (not necessary)
    tmpq = demos{i}; 
    
    % computing the first time derivative
    if ~iscell(time)
        tmpq_d = diff(tmpq,1,2)/dt;
    else
        tmpq_d = diff(tmpq,1,2)./repmat(diff(time{i}),dimq,1);
    end
    
    % trimming demonstrations
    ind = find(sqrt(sum(tmpq_d.*tmpq_d,1))>tol_cutting);
    tmpq = tmpq(:,min(ind):max(ind)+1);   
    
    if iscell(time)
        tmpt = time{i}(:,min(ind):max(ind)+1);
    end
    tmpq_d = tmpq_d(:,min(ind):max(ind));
    
    %saving the final point (target) of each demo
    xT(:,i) = robotplant.forward_kinematics(demos{i}(:,end));
    
    tmp_full = [tmpq;tmpq_d zeros(dimq,1)]; % all of the quantities to be placed in "Data"
    % note that the last qd is 0 because it's reached the goal
    
    tmp_full = [tmp_full; repmat(xT(:, i), 1, size(tmpq, 2))];
    
    
    % saving demos next to each other
    Data = [Data tmp_full];
    index = [index size(Data,2)+1];
end

end

