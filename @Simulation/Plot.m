function o = Plot(o, AGENT, TARGET, CLOCK)

% Target plot (FIGURE 1)
for iTarget = 1 : o.nTarget
    TARGET(iTarget).Plot();
    legend([get(legend(gca),'string'),TARGET(iTarget).plot.legend]);
end

% Agent plot (FIGURE 1)
for iAgent = 1 : o.nAgent
    AGENT(iAgent).Plot();
    legend([get(legend(gca),'string'),AGENT(iAgent).plot.legend]);

    AGENT(iAgent).MEASURE.Plot(AGENT(iAgent));
    legend([get(legend(gca),'string'),AGENT(iAgent).MEASURE.plot.legend]);
end

axis equal;

% Centralized Estimation plot
% FIGURE 2- : TARGET POSITION ERROR PLOT
% FIGURE 3-(nAGENT*nTarget+3): AGENT-TARGET ERROR PLOT
% o.CENTRAL_KF.Plot(AGENT,TARGET,CLOCK,o,'central');

% Individual Estimation plot
for iAgent = 1 : o.nAgent
    for iTarget = 1 : o.nTarget
        AGENT(iAgent).LOCAL_KF(iTarget).Plot(AGENT(iAgent),TARGET(iTarget),CLOCK,o,'local');
    end
end

end
