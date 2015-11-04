function o = Default( o, CLOCK ,iTarget)

% default setting for targets
% input : empty Target Class
%
% output : set Target Class

o.id = iTarget;
o.Ft = eye(2); 
o.Gt = CLOCK.dt*eye(2); 
o.Gu = zeros(2); % 2 state - 2 state
o.x = [0.1,0.1]';
o.hist.x = o.x; % store initial condition

o.Qt = diag([0.2; 0.2]); 

o.plot.color = 'b';
o.plot.marker = ['bo';'bx']; % start; end
o.plot.markersize = 10;
o.plot.linewidth = 3;

o.plot.legend = [{strcat('Target ',num2str(o.id))},...
    {strcat('Target ',num2str(o.id),' start')},...
    {strcat('Target ',num2str(o.id),' end')}];
    
end