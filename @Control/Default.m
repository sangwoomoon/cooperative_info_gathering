function o = Default (o, CLOCK)

o.u = 0; % zero IC


o.hist.u = o.u; % store control input (w.r.t. acceleration)
o.hist.stamp = CLOCK.ct; % store initialized time
o.hist.c = nan(2,1);

end