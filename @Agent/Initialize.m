function obj = Initialize ( obj, id, DynamicsOption, SensorOption, EstimationOption, FusionOption)

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
    case ('Static')
        obj.DYNAMICS = StaticDynamics();
    otherwise
        obj.DYNAMICS = Dynamics();
end

% initialize measurement class with respect to specified measurement model
switch (SensorOption)
    case ('RelPolar')
        obj.SENSOR = RelPolarSensor();
    case ('RelCart')
        obj.SENSOR = RelCartSensor();
    case ('RelCartBias')
        obj.SENSOR = RelCartBiasSensor();
    case ('InertCart')
        obj.SENSOR = InertCartSensor();
    otherwise
        obj.SENSOR = Sensor();
end

%obj.TA = TaskAllocation(TARGET, CLOCK); % Task Allocation sub-class

% initialize communication : could use sub-class with respect to comm
% package
obj.COMM = Communication(); 

% initialize estimator class with respect to specified estimation process
switch (EstimationOption)
    case ('KF')
        obj.ESTIMATOR = KalmanFilter();
    otherwise
        obj.ESTIMATOR = Estimator();
end

% initialize fusion class with respect to specified fusion process
switch (FusionOption)
    case ('CIWEP')
        obj.FUSION = CIWEP();
    otherwise
        obj.FUSION = Fusion();
end

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