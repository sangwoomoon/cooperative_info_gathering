function o = PredictTargetDynamics(o)

% predict with zero noise
o.x = o.Ft*o.x;

end