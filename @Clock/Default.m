function o = Default (o)

% default setting for simulation
% input : empty Clock Class
%
% output : set Clock Class

o.t0 = 0.1;

o.ct = 0;
o.dt = 0.1; % general time step for basic process;

o.delt.control = o.dt;
o.delt.target = o.dt;
o.delt.agent = o.dt;
o.delt.environment = o.dt;
o.delt.measurement = o.dt;
o.delt.communicate = o.dt;
o.delt.filter = o.dt;

o.nt = 200;
o.tf = o.dt*o.nt;

o.tvec = o.t0:o.dt:o.tf;

end