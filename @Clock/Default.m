function o = Default (o, t0, nt, SIM)

% default setting for simulation
% input : empty Clock Class
%
% output : set Clock Class

o.t0 = t0;

o.ct = 0;

o.nt = nt;
o.tf = o.nt;

o.tvec = o.t0:o.tf;

if SIM.bPlot == 1
    o.plot.h.title = title(['Iteration : ', num2str(0)]);
end

end