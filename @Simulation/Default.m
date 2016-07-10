function o = Default( o, nAgent, nSeed, bCentral, bPlot, bComm, mode )

% default setting for simulation
% input : empty Simulation Class
%
% output : set Simulation Class

o.bSim = 1;

o.nAgent = nAgent;

o.sRandom = rng(nSeed);

o.bPlot = bPlot;

o.bCentral = bCentral;

o.bComm = bComm;

o.hist.util = nan;

o.mode = mode;

end