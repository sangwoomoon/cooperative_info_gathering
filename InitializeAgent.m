function agent = InitializeAgent(iAgent, sim, speed)

field = sim.field;
flagPlot = sim.flagPlot;

% basic state setting
agent.id = iAgent;
agent.s = [-rand()*field.zoneLength(1)/8 -rand()*field.zoneLength(2)/8, 2*pi*rand speed]';
agent.hist.s = agent.s;

% agent display
if flagPlot
    
    % plotting parameter setting: blue, circle color
    agent.plot.clr = [0 rand() 1];
    agent.plot.marker = 'o';
    agent.plot.opaque = 0.1;
    
    % agent position plotting
    agent.plot.pos = ...
        plot(agent.s(1),agent.s(2),...
        agent.plot.marker,'LineWidth',2,'color',agent.plot.clr);
    agent.plot.id = ...
        text(agent.s(1),agent.s(2),...
        num2str(iAgent));
    
    % agent trajectory plotting
    agent.plot.path = animatedline(agent.s(1),agent.s(2),'color',agent.plot.clr);
end

end