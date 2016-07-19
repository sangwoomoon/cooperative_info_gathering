function o = Declare( o, nAgent, nTarget, nLandmark )

% default setting for simulation
% input : empty Simulation Class
%
% output : set Simulation Class

o.nTarget = nTarget;
o.nAgent = nAgent;
o.nLandmark = nLandmark;

o.sRandom = 333555532;

end