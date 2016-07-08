function o = Default( o, nAgent, nSeed, bPlot )

% default setting for simulation
% input : empty Simulation Class
%
% output : set Simulation Class

o.nAgent = nAgent;

o.sRandom = rng(nSeed);

o.bPlot = bPlot;

o.hist.util = nan;

end