function PlotPlanningResults(sim,option)

nSim = length(sim(1,:)); % number of MC simulation runs
mSim = length(sim(:,1)); % number of planning scheme

Hmean = zeros(mSim,sim(1,1).clock.nt+1);
Hvar = nan(mSim,sim(1,1).clock.nt+1);

% computate statistical information
for jSim = 1 : mSim
    for iSim = 1 : nSim
        
        % number of agents could be changed by condition
        nAgent = sim(jSim,1).nAgent;
        nTarget = sim(jSim,1).nTarget;
        
        for iAgent = 1 : nAgent
            for iTarget = 1 : nTarget
                Hmean(jSim,:) = Hmean(jSim,:) + sim(jSim,iSim).filter(iAgent,iTarget).hist.Hafter;
            end
        end
        % devide wrt agents & targets
        Hmean(jSim,:) = Hmean(jSim,:)./(nAgent*nTarget);
    end
    % devide wrt MC simulation runs
    Hmean(jSim,:) = Hmean(jSim,:)./nSim;
end

% results from planning W/O communication
figure(11),
plot(sim(1,1).clock.hist.time,Hmean); hold on;
switch option
    case 'planner'
        legend('random','mean tracking','MI(w/o comm-aware)','MI(w/ comm-aware)');
    case 'nA'
        legend('n=2','n=3','n=5','n=10');
end
xlabel('time [sec]');
ylabel('entropy [nats]');