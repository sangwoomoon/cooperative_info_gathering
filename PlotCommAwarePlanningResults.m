function PlotCommAwarePlanningResults(sim)

nSim = length(sim(1,:)); % number of MC simulation runs
nAgent = sim(1,1).nAgent;
nTarget = sim(1,1).nTarget;

for iSim = 1 : nSim
    
    % without comm planning
    for iAgent = 1 : nAgent
        
        I = zeros(size(sim(1,iSim).clock.hist.time(2:end)));
        for iTarget = 1 : nTarget
            I = I + sim(1,iSim).filter(iAgent,iTarget).hist.I(2:end);
        end
        planErr(1,iSim,iAgent,:) = sim(1,iSim).planner(iAgent).hist.I(2:end)-I;
        
        % with comm planning
        I = zeros(size(sim(1,iSim).clock.hist.time(2:end)));
        for iTarget = 1 : nTarget
            I = I + sim(2,iSim).filter(iAgent,iTarget).hist.I(2:end);
        end
        planErr(2,iSim,iAgent,:) = sim(2,iSim).planner(iAgent).hist.I(2:end)-I;
        
    end
    
end



% results from planning W/O communication
figure(11),
% plot(sim(1,1).clock.hist.time,sim(1,1).planner(1).hist.Hbefore,sim(1,1).clock.hist.time,sim(1,1).filter(1).hist.Hbefore), hold on;
plot(sim(1,1).clock.hist.time,sim(1,1).planner(1).hist.I-sim(1,1).filter(1).hist.I,'r-'); hold on;

% results from planning W/ communication
% plot(sim(2,1).clock.hist.time,sim(2,1).planner(1).hist.Hbefore,sim(2,1).clock.hist.time,sim(2,1).filter(1).hist.Hbefore), hold on;
plot(sim(2,1).clock.hist.time,sim(2,1).planner(1).hist.I-sim(2,1).filter(1).hist.I,'b-');