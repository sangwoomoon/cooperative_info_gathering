function o = Default( o )

% default setting for simulation
% input : empty Simulation Class
%
% output : set Simulation Class

o.nTarget = 2;
o.nAgent = 3;

o.sRandom = rng(333555533);

end