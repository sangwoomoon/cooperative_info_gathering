function o = PredictAgentDynamics(o)

% prediction with zero noise input
o.s = o.Fp*o.s + o.Gu*o.u; 

end