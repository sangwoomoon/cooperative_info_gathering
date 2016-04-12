function o = Default( o, nAgent, nTarget )

% default setting for simulation
% input : empty Simulation Class
%
% output : set Simulation Class

o.nTarget = nTarget;
o.nAgent = nAgent;
% o.nLandMark = nLandMark;

o.sRandom = rng(333555532);

end