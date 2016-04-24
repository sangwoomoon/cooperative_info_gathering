function o = Plot(o, AGENT, TARGET, CLOCK)

% Target plot (FIGURE 1)
figure(1), hold on;
for iTarget = 1 : o.nTarget
    TARGET(iTarget).Plot();
    legend([get(legend(gca),'string'),TARGET(iTarget).plot.legend]);
end

% Agent plot (FIGURE 1)
for iAgent = 1 : o.nAgent
    AGENT(iAgent).Plot();
    legend([get(legend(gca),'string'),AGENT(iAgent).plot.legend]);
%     for iTarget = 1 : o.nTarget
%         AGENT(iAgent).MEASURE(iTarget).Plot(AGENT(iAgent),iTarget);
%         if AGENT(iAgent).TA.bTasklist(iTarget) == 1
%             legend([get(legend(gca),'string'),AGENT(iAgent).MEASURE(iTarget).plot.legend]);
%         end
%     end
end

% Voronoi Centroid plot (FIGURE 1)
o.LLOYD.Plot('profile');

xlabel('East (m)');
ylabel('North (m)');
axis equal;

% Centralized Estimation plot
% FIGURE 2- : TARGET POSITION ERROR PLOT
% FIGURE 3-(nAGENT*nTarget+3): AGENT-TARGET ERROR PLOT
% o.CENTRAL_KF.Plot(AGENT,TARGET,CLOCK,o,'central');

% Individual Estimation plot
for iAgent = 1 : o.nAgent
    AGENT(iAgent).LOCAL_EKF.Plot(AGENT(iAgent),TARGET,CLOCK,o,'local');
%     AGENT(iAgent).FDDF_KF.Plot(AGENT(iAgent),TARGET,CLOCK,o,'fDDF');
end

end
