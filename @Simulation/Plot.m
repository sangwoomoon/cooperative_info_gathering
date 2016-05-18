function o = Plot(o, AGENT, TARGET, CENTRAL_KF, CLOCK)

% Target plot (FIGURE 1)
for iTarget = 1 : o.nTarget
    TARGET(iTarget).Plot();
    legend([get(legend(gca),'string'),TARGET(iTarget).plot.legend]);
end

% Agent plot (FIGURE 1)
for iAgent = 1 : o.nAgent
    AGENT(iAgent).Plot();
    legend([get(legend(gca),'string'),AGENT(iAgent).plot.legend]);
    for iTarget = 1 : o.nTarget
        AGENT(iAgent).MEASURE(iTarget).Plot(AGENT(iAgent));
        legend([get(legend(gca),'string'),AGENT(iAgent).MEASURE(iTarget).plot.legend]);
    end
end
axis equal;

% Centralized Estimation plot
% FIGURE 2- : TARGET POSITION ERROR PLOT
% FIGURE 3-(nAGENT*nTarget+3): AGENT-TARGET ERROR PLOT
CENTRAL_KF.Plot(AGENT,TARGET,CLOCK,o,'central');

% Individual Estimation plot
for iAgent = 1 : o.nAgent
    AGENT(iAgent).LOCAL_KF.Plot(AGENT(iAgent),TARGET,CLOCK,o,'local');
    AGENT(iAgent).FDDF_KF.Plot(AGENT(iAgent),TARGET,CLOCK,o,'fDDF');
end

end
