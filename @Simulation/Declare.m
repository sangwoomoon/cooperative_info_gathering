function obj = Declare( obj, nSim, nAgent, nTarget, nLandmark )

% default setting for simulation
% input : empty Simulation Class
%
% output : set Simulation Class with basic parameters

obj.nSim = nSim;
obj.nTarget = nTarget;
obj.nAgent = nAgent;
obj.nLandmark = nLandmark;

for iSimSeed = 1 : nSim
    obj.sRandom(iSimSeed) = iSimSeed;
end

% initialize current figure
obj.iFigure = 1;


end