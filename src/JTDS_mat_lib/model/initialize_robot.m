function robot = initialize_robot(A, D, Alpha, Qmin, Qmax, plotopt)
%INITIALIZE_ROBOT Creates a robot object using Peter Corke's Robotics Toolbox 
%   This object can be used to generate jacobians and forward kinematics,
%   and to pass around robot parameters. The first 3 parameters are column
%   vectors of Denavit-Hartenberg parameters (with the "offset q" assumed to be 0).
%   The last two are minimum and maximum joint angles with respect to the
%   default "0" angle.
%   Plotopt is a struct containing plotting options as specified in Corke's
%   robot-toolkit
    qdim = length(A);
    if nargin < 5% if joint limits unspecified, set them to the max possible
        Qmin = 2*pi*ones(qdim, 1);
        Qmax = -2*pi*ones(qdim, 1);
        warning('No joint limits specified, may cause unwanted behavior if using limit-enforcement methods.');
    end
    
    for i = 1:qdim
        L(i) = Link('d', D(i), 'a', A(i), 'alpha', Alpha(i), 'standard', 'revolute', 'qlim', [Qmin(i) Qmax(i)]);
    end
    robot = SerialLink(L, 'name','my_robot');
    if nargin == 6
        robot.plotopt = plotopt;
    else
        robot.plotopt = {'noshadow','nojaxes', 'nowrist', 'noname','jointdiam', 2, 'basewidth', 1, 'linkcolor',0.7*[1,1,1], 'ortho','noshading','notiles','jointcolor',0.6*[1,1,1]};
        %,'nojaxes', 'nowrist', 'noname'
    end
end
