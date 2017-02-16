function obj = Plot(obj, AGENT, TARGET, ENV, CLOCK)

%% sim performance plot
% 
% figure(obj.iFigure)
% boxplot([obj.cost(1,:)',obj.cost(2,:)',obj.cost(3,:)'],{'Central','MMNB Fusion','Diag. Fusion'});
% ylabel('max.variance','fontsize',20);
% set(gca,'fontsize',20)
% 
% obj.iFigure = obj.iFigure + 1;

%% one simulation plot (for final sim run)

% Environment (Landmark) plot (FIGURE 1)
figure(obj.iFigure)
for iLandmark = 1 : obj.nLandmark
    ENV.LANDMARK.DYNAMICS.Plot(ENV.LANDMARK(iLandmark)); % TARGET input for plotting option (TARGET.plot is better)
    legend([get(legend(gca),'string'),ENV.LANDMARK(iLandmark).plot.legend]);
end

% Target plot (FIGURE 1)
for iTarget = 1 : obj.nTarget
    TARGET(iTarget).DYNAMICS.Plot(TARGET(iTarget)); % TARGET input for plotting option (TARGET.plot is better)
    legend([get(legend(gca),'string'),TARGET(iTarget).plot.legend]);
end

% Agent plot (FIGURE 1)
for iAgent = 1 : obj.nAgent
    AGENT(iAgent).DYNAMICS.Plot(AGENT(iAgent)); % AGENT input for plotting option
    legend([get(legend(gca),'string'),AGENT(iAgent).plot.legend]);
end

% Measurement plot (FIGURE 1)
for iAgent = 1 : obj.nAgent
    AGENT(iAgent).SENSOR.Plot(ENV);
end
axis equal;
obj.iFigure = obj.iFigure+1;

% communication status plot (FIGURE 2)
figure(obj.iFigure)
obj.NETWORK.Plot(CLOCK);

% Centralized Estimation plot
% FIGURE 2- : TARGET POSITION ERROR PLOT
% FIGURE 3-(nAGENT*nTarget+3): AGENT-TARGET ERROR PLOT
obj.ESTIMATOR.Plot(AGENT,TARGET,CLOCK,obj,'central');

% Individual Estimation plot
for iAgent = 1 : obj.nAgent
    AGENT(iAgent).ESTIMATOR(1).Plot(AGENT(iAgent),TARGET,CLOCK,obj,'local'); hold on; % MMNB Method
    % AGENT(iAgent).ESTIMATOR(2).Plot(AGENT(iAgent),TARGET,CLOCK,obj,'local'); hold on; % diag Method
end

end
