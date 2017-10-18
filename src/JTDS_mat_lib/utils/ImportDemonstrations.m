function [Qs, Ts] = ImportDemonstrations(filename)
%IMPORTDEMONSTRATION Converts from a demonstration text file
%   to a set of joint positions Q and times T
    file = load(filename);
    N = length(file.data);
    Qs = cell(N, 1);
    Ts = cell(N, 1);
    for i = 1:N
        Qs{i} = file.data{i}.kuka.joint_states(1:end-1, :);
        Ts{i} = file.data{i}.kuka.joint_states(end, :);
    end
end

