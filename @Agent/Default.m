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
        o.DYNAMICS = LinearDynamics(o.CONTROL, CLOCK, o.id, 'Agent');
    case ('LinearBias')
        o.DYNAMICS = LinearBiasDynamics(o.CONTROL, CLOCK, o.id);
    case ('Dubins')
        o.DYNAMICS = DubinsDynamics(o.CONTROL, CLOCK, o.id, 'Agent');
    otherwise
        o.DYNAMICS = Dynamics(o.CONTROL, CLOCK, o.id, 'Agent');
end


% initialize measurement class with respect to specified measurement model
for iTarget = 1 : length(TARGET)
    MEASURE(iTarget) = Measurement(TARGET(iTarget), CLOCK, o.id);
end
o.MEASURE = MEASURE;


o.TA = TaskAllocation(TARGET, CLOCK); % Task Allocation sub-class
o.COMM = Communication(SIMULATION, CLOCK); % Communication sub-class


end