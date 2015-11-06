function o = Default( o, SIMULATION, CLOCK ,iTarget)

% default setting for targets
% input : empty Target Class
%
% output : set Target Class

% InfoTech Version : consider states with bias,
% Target also has dynamics with constant velocity.


o.id = iTarget;
o.Ft = blkdiag(eye(2*SIMULATION.nAgent),[1 CLOCK.dt; 0 1],[1 CLOCK.dt; 0 1]);

Gt_d = [0.5*CLOCK.dt^2             0   ;
            CLOCK.dt               0   ;
                   0    0.5*CLOCK.dt^2 ;
                   0        CLOCK.dt  ]; % only for target part (beware that w/o bias!)
               
o.Gt = [zeros(2*SIMULATION.nAgent,2);Gt_d];
 
o.Gu = zeros(2); % 2 state - 2 state

o.x = [0.4,0.4,0.2,0.2,1.0,0.1,1.0,0.1]'; % just for default (may be changed in the main script)
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