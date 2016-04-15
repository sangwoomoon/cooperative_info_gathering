function o = Default ( o, TARGET, ENVIRONMENT, SIMULATION, CLOCK, iAgent)


SIMULATION.sRandom; % random seed setting

% default setting for targets
% input : empty Agent Class
%
% output : set Agent Class

o.id = iAgent;

o.Fp = blkdiag([1 CLOCK.dt; 0 1],[1 CLOCK.dt; 0 1]);

o.Gamp = [0.5*CLOCK.dt^2             0   ;
              CLOCK.dt               0   ;
                     0    0.5*CLOCK.dt^2 ;
                     0        CLOCK.dt  ]; 

o.Gu = [  0.5*CLOCK.dt^2             0   ;
              CLOCK.dt               0   ;
                     0    0.5*CLOCK.dt^2 ;
                     0        CLOCK.dt  ]; 
                 
% (initially) stationary, randomized location within given environment
o.s = [ENVIRONMENT.xlength(1)+(ENVIRONMENT.xlength(2)-ENVIRONMENT.xlength(1))*rand(1);...
                                                                                    0;...
       ENVIRONMENT.ylength(1)+(ENVIRONMENT.ylength(2)-ENVIRONMENT.ylength(1))*rand(1);...
                                                                                    0];
                                                                                
o.Qp = diag([0.05 0.05]);

o.hist.s = o.s; % store initial condition
o.hist.stamp = 0; % store initialized time

o.TA = TaskAllocation(TARGET, CLOCK); % Task Allocation sub-class
o.COMM = Communication(SIMULATION, CLOCK); % Communication sub-class

for iTarget = 1 : length(TARGET)
    MEASURE(iTarget) = Measurement(TARGET(iTarget), CLOCK, o.id); % Measurement sub-class
end

o.MEASURE = MEASURE;
o.CONTROL = Control(CLOCK); % Control sub-class

o.plot.statecolor = rand(1,3);
o.plot.marker = ['o';'x']; % start; end
o.plot.markersize = 10;
o.plot.line = '*';
o.plot.linewidth = 1;

o.plot.legend = [{strcat('Agent ',num2str(o.id))},...
    {strcat('Agent ',num2str(o.id),' start')},...
    {strcat('Agent ',num2str(o.id),' end')}];

end