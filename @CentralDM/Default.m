
function o = Default(o, SIMULATION, AGENT)

for iAgent = 1 : SIMULATION.nAgent
   o.s(:,iAgent) = AGENT(iAgent).s; 
end

o.hist.s = o.s;

o.util = nan;
o.hist.util = o.util;

o.alpha = AGENT(1).alpha;
o.beta = AGENT(1).beta;

o.eta = AGENT(1).eta;
o.gamma = AGENT(1).gamma;

if SIMULATION.bPlot == 1
    
    for iAgent = 1 : SIMULATION.nAgent
        o.plot(iAgent).h.num = text(o.s(1,iAgent),o.s(2,iAgent),num2str(AGENT(iAgent).id),'color','r');
        
        o.plot(iAgent).statecolor = AGENT(iAgent).plot.statecolor;
        o.plot(iAgent).marker = ['x';'^']; % start; current
        o.plot(iAgent).markersize = 10;
        o.plot(iAgent).line = '.';
        o.plot(iAgent).linewidth = 1;
        
        if AGENT(iAgent).bDFC == 0
            o.plot(iAgent).h.start = plot(o.s(1,iAgent),o.s(2,iAgent),o.plot(iAgent).marker(1),'MarkerSize',o.plot(iAgent).markersize,'color',o.plot(iAgent).statecolor);
            o.plot(iAgent).h.curr = plot(o.s(1,iAgent),o.s(2,iAgent),o.plot(iAgent).marker(2),'MarkerSize',o.plot(iAgent).markersize,'MarkerEdgeColor',o.plot(iAgent).statecolor,'MarkerFaceColor',o.plot(iAgent).statecolor);
        else
            o.plot(iAgent).h.start = plot(o.s(1,iAgent),o.s(2,iAgent),o.plot(iAgent).marker(1),'MarkerSize',o.plot(iAgent).markersize,'color',o.plot(iAgent).statecolor);
            o.plot(iAgent).h.curr = plot(o.s(1,iAgent),o.s(2,iAgent),'square','MarkerSize',o.plot(iAgent).markersize,'MarkerEdgeColor',o.plot(iAgent).statecolor,'MarkerFaceColor',o.plot(iAgent).statecolor);
        end
        
        o.plot(iAgent).h.path = plot(o.s(1,iAgent),o.s(2,iAgent),o.plot(iAgent).line,'LineWidth',o.plot(iAgent).linewidth,'color',ones(1,3)*.8);
    end
    
end

end