function obj = Declare ( obj, id, TARGET, SIMULATION, CLOCK, DynamicsOption, SensorOption)

% default setting for targets
% input : empty Agent Class
%
% output : set Agent Class

obj.id = id;

% initialize control class
obj.CONTROL = Control(); % Control sub-class

% initialize dynamics class with respect to specified dynamics model
switch (DynamicsOption)
    case ('Linear')
        obj.DYNAMICS = LinearDynamics();
    case ('Dubins')
        obj.DYNAMICS = DubinsDynamics();
    otherwise
        obj.DYNAMICS = Dynamics();
end

% initialize measurement class with respect to specified measurement model
switch (SensorOption)
    case ('RelPolarCoord')
        obj.SENSOR = RelPolarCoordSensor();
    case ('RelCartCoord')
        obj.SENSOR = RelCartCoordSensor();
    case ('InertCartCoord')
        obj.SENSOR = InertCartCoordSensor();
    otherwise
        obj.SENSOR = Sensor();
end



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