function o = Default ( o, TARGET, ENVIRONMENT, SIMULATION, CLOCK, iAgent)

% default setting for targets
% input : empty Agent Class
%
% output : set Agent Class

o.id = iAgent;

o.Fp = blkdiag(eye(2),[1 CLOCK.dt; 0 1],[1 CLOCK.dt; 0 1]);

o.Gamp = [    CLOCK.dt               0   ;
                    0          CLOCK.dt  ;
          0.5*CLOCK.dt^2             0   ;
              CLOCK.dt               0   ;
                     0    0.5*CLOCK.dt^2 ;
                     0        CLOCK.dt  ]; % make the separate Q matrix

o.Gu = [            0                0   ;
                    0                0   ;
          0.5*CLOCK.dt^2             0   ;
              CLOCK.dt               0   ;
                     0    0.5*CLOCK.dt^2 ;
                     0        CLOCK.dt  ]; % make the separate Q matrix


o.Qp = diag([0.01 0.01]);


o.TA = TaskAllocation(TARGET, CLOCK); % Task Allocation sub-class
o.COMM = Communication(SIMULATION, CLOCK); % Communication sub-class

for iTarget = 1 : length(TARGET)
    MEASURE(iTarget) = Measurement(TARGET(iTarget), CLOCK, o.id); % Measurement sub-class
end

o.MEASURE = MEASURE;
o.CONTROL = Control(o, TARGET, ENVIRONMENT); % Control sub-class

o.plot.statecolor = rand(1,3);
o.plot.marker = ['o';'x']; % start; end
o.plot.markersize = 10;
o.plot.line = '--';
o.plot.linewidth = 3;

o.plot.legend = [{strcat('Agent ',num2str(o.id))},...
    {strcat('Agent ',num2str(o.id),' start')},...
    {strcat('Agent ',num2str(o.id),' end')}];

end