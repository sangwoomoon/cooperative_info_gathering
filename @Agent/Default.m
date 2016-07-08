function o = Default ( o, ENVIRONMENT, SIMULATION, CLOCK, iAgent, iDFC)


SIMULATION.sRandom; % random seed setting

% default setting for targets
% input : empty Agent Class
%
% output : set Agent Class

o.id = iAgent;

if o.id == iDFC
    o.bDFC = 1;
else
    o.bDFC = 0;
end

                 
% (initially) randomized location within given environment
o.s = [ENVIRONMENT.xlength(1)+(ENVIRONMENT.xlength(2)-ENVIRONMENT.xlength(1))*rand(1);...
       ENVIRONMENT.ylength(1)+(ENVIRONMENT.ylength(2)-ENVIRONMENT.ylength(1))*rand(1)];                                                                   

o.alpha = 1;
o.beta = 1;

o.eta = 5;
o.gamma = 5;
   
o.util = nan; % utility

o.hist.s = o.s; % store initial condition
o.hist.stamp = 0; % store initialized time

o.hist.util = o.util;

o.COMM = Communication(SIMULATION, CLOCK); % Communication sub-class

if SIMULATION.bPlot == 1
    
    o.plot.h.num = text(o.s(1),o.s(2),num2str(o.id));
    
    o.plot.statecolor = rand(1,3);
    o.plot.marker = ['x';'o']; % start; current
    o.plot.markersize = 10;
    o.plot.line = '-';
    o.plot.linewidth = 1;
    
    if o.bDFC == 0
        o.plot.h.start = plot(o.s(1),o.s(2),o.plot.marker(1),'MarkerSize',o.plot.markersize,'color',o.plot.statecolor);
        o.plot.h.curr = plot(o.s(1),o.s(2),o.plot.marker(2),'MarkerSize',o.plot.markersize,'MarkerEdgeColor',o.plot.statecolor,'MarkerFaceColor',o.plot.statecolor);
    else
        o.plot.h.start = plot(o.s(1),o.s(2),o.plot.marker(1),'MarkerSize',o.plot.markersize,'color',o.plot.statecolor);
        o.plot.h.curr = plot(o.s(1),o.s(2),'square','MarkerSize',o.plot.markersize,'MarkerEdgeColor',o.plot.statecolor,'MarkerFaceColor',o.plot.statecolor);
    end
    
    o.plot.h.path = plot(o.s(1),o.s(2),o.plot.line,'LineWidth',o.plot.linewidth,'color',ones(1,3)*.8);
    o.plot.h.util = plot(o.util,'-','color',rand(1,3)); % N/A
    
    
    o.plot.legend = [{strcat('Agent ',num2str(o.id))},...
        {strcat('Agent ',num2str(o.id),' start')},...
        {strcat('Agent ',num2str(o.id),' end')}];
    
end

end