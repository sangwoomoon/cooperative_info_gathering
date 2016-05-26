function o = Default (o, t0, dt, nt, FDDFt)

% default setting for simulation
% input : empty Clock Class
%
% output : set Clock Class

o.dt_sym = sym('dt_sym');

o.t0 = t0;

o.ct = 0;
o.dt = dt; % general time step for basic process;

o.delt.control = o.dt;
o.delt.target = o.dt;
o.delt.agent = o.dt;
o.delt.environment = o.dt;
o.delt.measurement = o.dt;
o.delt.communicate = o.dt;
o.delt.filter = o.dt;
o.delt.FDDF = FDDFt/o.dt;

o.nt = nt;
o.tf = o.dt*o.nt;

o.tvec = o.t0:o.dt:o.tf;

end