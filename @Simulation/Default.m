function o = Default( o, nAgent, nSeed, bCentral, bPlot, bComm )

% default setting for simulation
% input : empty Simulation Class
%
% output : set Simulation Class

o.nAgent = nAgent;

o.sRandom = rng(nSeed);

o.bPlot = bPlot;

o.bCentral = bCentral;

o.bComm = bComm;

o.hist.util = nan;

end