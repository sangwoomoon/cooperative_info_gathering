function o = Default( o, ENVIRONMENT, iTarget, option)

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
        o.DYNAMICS = LinearDynamics();
        o.CONTROL.u = zeros(2,1); % set as zero input for random walk of target movement
    case ('Dubins')
        o.DYNAMICS = DubinsDynamics();
        o.CONTROL.u = zeros(2,1); % set as zero input for random walk of target movement
    otherwise
        o.DYNAMICS = Dynamics();
end

% plotting options setting
o.plot.statecolor = rand(1,3);
o.plot.marker = ['o';'^']; % start; end
o.plot.markersize = 7;
o.plot.line = '.-';
o.plot.linewidth = 5;

o.plot.legend = [{strcat('Target ',num2str(o.id))},...
    {strcat('Target ',num2str(o.id),' start')},...
    {strcat('Target ',num2str(o.id),' end')}];

    
end