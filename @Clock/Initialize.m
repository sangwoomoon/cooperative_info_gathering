function obj = Initialize( obj, t0, dt, nt, FDDFt )
%INITIALIZE initialize clock properties for simulation
% input : empty Clock Class
%
% output : set Clock Class

obj.t0 = t0;

obj.ct = 0;
obj.dt = dt; % general time step for basic process;

obj.delt.control = obj.dt;
obj.delt.target = obj.dt;
obj.delt.agent = obj.dt;
obj.delt.environment = obj.dt;
obj.delt.measurement = obj.dt;
obj.delt.communicate = obj.dt;
obj.delt.filter = obj.dt;
obj.delt.FDDF = FDDFt/obj.dt;

obj.nt = nt;
obj.tf = obj.dt*obj.nt;

obj.tvec = obj.t0:obj.dt:obj.tf;



end

