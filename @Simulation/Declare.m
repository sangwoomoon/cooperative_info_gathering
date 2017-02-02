function obj = Declare( obj, nSim, nAgent, nTarget, nLandmark, EstimationOption, NetworkOption )

% default setting for simulation
% input : empty Simulation Class
%
% output : set Simulation Class

obj.nSim = nSim;
obj.nTarget = nTarget;
obj.nAgent = nAgent;
obj.nLandmark = nLandmark;

for iSimSeed = 1 : nSim
    obj.sRandom(iSimSeed) = iSimSeed;
end

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
    case ('Bernoulli')
        obj.NETWORK = BernoulliProbNetwork();
    otherwise
        obj.NETWORK = Network();
end

% initialize current figure
obj.iFigure = 1;


end