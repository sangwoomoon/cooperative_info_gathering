function o = Default ( o, TARGET, ENVIRONMENT, SIMULATION, CLOCK, iAgent, option)

% default setting for targets
% input : empty Agent Class
%
% output : set Agent Class

o.id = iAgent;

% initialize control class
o.CONTROL = Control(ENVIRONMENT); % Control sub-class

% initialize dynamics class with respect to specified dynamics model
switch (option)
    case ('Linear')
        o.DYNAMICS = LinearDynamics();
    case ('LinearBias')
        o.DYNAMICS = LinearBiasDynamics();
    case ('Dubins')
        o.DYNAMICS = DubinsDynamics();
    otherwise
        o.DYNAMICS = Dynamics();
end


% initialize measurement class with respect to specified measurement model
for iTarget = 1 : length(TARGET)
    MEASURE(iTarget) = Measurement(TARGET(iTarget), CLOCK, o.id);
end
o.MEASURE = MEASURE;


o.TA = TaskAllocation(TARGET, CLOCK); % Task Allocation sub-class
o.COMM = Communication(SIMULATION, CLOCK); % Communication sub-class

% plotting options setting
o.plot.statecolor = rand(1,3);
o.plot.marker = ['o';'x']; % start; end
o.plot.markersize = 7;
o.plot.line = '--';
o.plot.linewidth = 5;

o.plot.legend = [{strcat('Agent ',num2str(o.id))},...
    {strcat('Agent ',num2str(o.id),' start')},...
    {strcat('Agent ',num2str(o.id),' end')}];


end