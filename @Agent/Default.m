function obj = Default ( obj, TARGET, ENVIRONMENT, SIMULATION, CLOCK, iAgent, DyanmicsOption, MeasureOption)

% default setting for targets
% input : empty Agent Class
%
% output : set Agent Class

obj.id = iAgent;

% initialize control class
obj.CONTROL = Control(ENVIRONMENT); % Control sub-class

% initialize dynamics class with respect to specified dynamics model
switch (DyanmicsOption)
    case ('Linear')
        obj.DYNAMICS = LinearDynamics();
    case ('LinearBias')
        obj.DYNAMICS = LinearBiasDynamics();
    case ('Dubins')
        obj.DYNAMICS = DubinsDynamics();
    otherwise
        obj.DYNAMICS = Dynamics();
end

% initialize measurement class with respect to specified measurement model
for iTarget = 1 : length(TARGET)
    switch (MeasureOption)
        case ('Linear')
            MEASURE(iTarget) = LinearMeasure();
        case ('RangeBearing')
            MEASURE(iTarget) = RangeBearingMeasure();
        case ('RangeRate')
            MEASURE(iTarget) = RangeRateMeasure();
        otherwise
            MEASURE(iTarget) = Measurement();
    end
end
obj.MEASURE = MEASURE;


obj.TA = TaskAllocation(TARGET, CLOCK); % Task Allocation sub-class
obj.COMM = Communication(SIMULATION, CLOCK); % Communication sub-class

% plotting options setting
obj.plot.statecolor = rand(1,3);
obj.plot.marker = ['o';'x']; % start; end
obj.plot.markersize = 7;
obj.plot.line = '--';
obj.plot.linewidth = 5;

obj.plot.legend = [{strcat('Agent ',num2str(obj.id))},...
    {strcat('Agent ',num2str(obj.id),' start')},...
    {strcat('Agent ',num2str(obj.id),' end')}];


end