function obj = Declare( obj, targetID, option)

% default setting for targets
% input : empty Target Class
%
% output : set Target Class

% InfoTech Version : consider states with bias,
% Target also has dynamics with constant velocity.


obj.id = targetID;

% initialize control class
obj.CONTROL = Control(); % Control sub-class

% initialize dynamics class with respect to specified dynamics model
switch (option)
    case ('Linear')
        obj.DYNAMICS = LinearDynamics();
        obj.CONTROL.u = zeros(2,1); % set as zero input for random walk of target movement
    case ('Dubins')
        obj.DYNAMICS = DubinsDynamics();
        obj.CONTROL.u = zeros(2,1); % set as zero input for random walk of target movement
    case ('Static')
        obj.DYNAMICS = StaticDynamics();
    otherwise
        obj.DYNAMICS = Dynamics();
end

% plotting options setting
obj.plot.statecolor = rand(1,3);
obj.plot.marker = ['o';'^']; % start; end
obj.plot.markersize = 7;
obj.plot.line = '.-';
obj.plot.linewidth = 5;

switch (obj.id)
    case ('LANDMARK') % if it is landmark in the environment
        obj.plot.legend = [{strcat(num2str(obj.id))},...
            {strcat(num2str(obj.id),' start')},...
            {strcat(num2str(obj.id),' end')}];
    otherwise % if it is target
        obj.plot.legend = [{strcat('Target ',num2str(obj.id))},...
            {strcat('Target ',num2str(obj.id),' start')},...
            {strcat('Target ',num2str(obj.id),' end')}];
end

    
end