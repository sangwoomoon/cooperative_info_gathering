function o = Default( o, CLOCK ,iTarget)

% default setting for targets
% input : empty Target Class
%
% output : set Target Class

% InfoTech Version : consider states with bias,
% Target also has dynamics with constant velocity.


o.id = iTarget;
o.Ft = blkdiag([1 CLOCK.dt; 0 1],[1 CLOCK.dt; 0 1]);

o.Gt = [0.5*CLOCK.dt^2             0   ;
            CLOCK.dt               0   ;
                   0    0.5*CLOCK.dt^2 ;
                   0        CLOCK.dt  ]; % only for target part (beware that w/o bias!)
               
 
o.Gu = zeros(2); % 2 state - 2 state

o.x = [1.0,0.1,1.0,0.1]'; % just for default (may be changed in the main script)
o.hist.x = o.x; % store initial condition

o.Qt = diag([0.05; 0.05]); 

o.plot.statecolor = rand(1,3);
o.plot.marker = ['o';'x']; % start; end
o.plot.markersize = 10;
o.plot.linewidth = 3;

o.plot.legend = [{strcat('Target ',num2str(o.id))},...
    {strcat('Target ',num2str(o.id),' start')},...
    {strcat('Target ',num2str(o.id),' end')}];
    
end