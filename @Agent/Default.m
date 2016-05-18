function o = Default ( o, TARGET, ENVIRONMENT, SIMULATION, CLOCK, iAgent)

% default setting for targets
% input : empty Agent Class
%
% output : set Agent Class

o.id = iAgent;

o.CONTROL = Control(o, TARGET, ENVIRONMENT); % Control sub-class
o.DYNAMICS = Dynamics(SIMULATION, CLOCK); % Agent Dynamics sub-class
o.TA = TaskAllocation(TARGET, CLOCK); % Task Allocation sub-class
o.COMM = Communication(SIMULATION, CLOCK); % Communication sub-class

for iTarget = 1 : length(TARGET)
    MEASURE(iTarget) = Measurement(TARGET(iTarget), CLOCK, o.id); % Measurement sub-class
end

o.MEASURE = MEASURE;

o.plot.statecolor = rand(1,3);
o.plot.marker = ['o';'x']; % start; end
o.plot.markersize = 10;
o.plot.line = '--';
o.plot.linewidth = 3;

o.plot.legend = [{strcat('Agent ',num2str(o.id))},...
    {strcat('Agent ',num2str(o.id),' start')},...
    {strcat('Agent ',num2str(o.id),' end')}];

end