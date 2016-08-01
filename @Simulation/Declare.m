function obj = Declare( obj, nAgent, nTarget, nLandmark, EstimationOption, NetworkOption )

% default setting for simulation
% input : empty Simulation Class
%
% output : set Simulation Class

obj.nTarget = nTarget;
obj.nAgent = nAgent;
obj.nLandmark = nLandmark;

obj.sRandom = 333555532;

% initialize estimator class with respect to specified estimation process
switch (EstimationOption)
    case ('KF')
        obj.ESTIMATOR = KalmanFilter();
    otherwise
        obj.ESTIMATOR = Estimator();
end

% initialize network (amorphous) class with respect to specified network process
switch (NetworkOption)
    case ('Disk')
        obj.NETWORK = DiskModelNetwork();
    otherwise
        obj.NETWORK = Network();
end

% initialize current figure
obj.iFigure = 1;

end