function o = Default( o, CLOCK, ENVIRONMENT, iTarget, option)

% default setting for targets
% input : empty Target Class
%
% output : set Target Class

% InfoTech Version : consider states with bias,
% Target also has dynamics with constant velocity.


o.id = iTarget;

% initialize control class
o.CONTROL = Control(ENVIRONMENT); % Control sub-class

% initialize dynamics class with respect to specified dynamics model
switch (option)
    case ('Linear')
        o.DYNAMICS = LinearDynamics(o.CONTROL, CLOCK, o.id, 'Target');
        o.CONTROL.u = zeros(1,2); % set as zero input for random walk of target movement
    case ('Dubins')
        o.DYNAMICS = DubinsDynamics(o.CONTROL, CLOCK, o.id, 'Target');
        o.CONTROL.u = zeros(1,2); % set as zero input for random walk of target movement
    otherwise
        o.DYNAMICS = Dynamics(o.CONTROL, CLOCK, o.id, 'Target');
end
    
end