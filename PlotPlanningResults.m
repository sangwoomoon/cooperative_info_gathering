function PlotPlanningResults(sim,option)

nSim = length(sim(1,:)); % number of MC simulation runs
mSim = length(sim(:,1)); % number of planning scheme
nPt = sim(1,1).filter(1,1).nPt; % number of particles

Hmean = zeros(mSim,sim(1,1).clock.nt+1);
rmse = zeros(mSim,sim(1,1).clock.nt+1);

% computate statistical information
for jSim = 1 : mSim
    for iSim = 1 : nSim
        
        % number of agents could be changed by condition
        nAgent = sim(jSim,1).nAgent;
        nTarget = sim(jSim,1).nTarget;
        
        for iAgent = 1 : nAgent
            for iTarget = 1 : nTarget
                rmse(jSim,:) = rmse(jSim,:) + ...
                            sqrt(sum(squeeze(sim(jSim,iSim).filter(iAgent,iTarget).hist.w(1,:,:)).*...
                            squeeze((sim(jSim,iSim).filter(iAgent,iTarget).hist.pt(1,:,:) - sim(jSim,iSim).target(iTarget).x(1,:))).^2 + ...
                            squeeze((sim(jSim,iSim).filter(iAgent,iTarget).hist.pt(2,:,:) - sim(jSim,iSim).target(iTarget).x(2,:))).^2, 1)/nPt);
                Hmean(jSim,:) = Hmean(jSim,:) + sim(jSim,iSim).filter(iAgent,iTarget).hist.Hafter;
            end
        end
    end
    % devide wrt agents, targets, and MC simulation runs
    rmse(jSim,:) = rmse(jSim,:)./(nAgent*nTarget*nSim);
    Hmean(jSim,:) = Hmean(jSim,:)./(nAgent*nTarget*nSim);
end

% results from planning: entropy variation
figure(11),
plot(sim(1,1).clock.hist.time,Hmean); hold on;
switch option
    case 'planner'
        legend('random','w/o comm-aware','disjoint','Gaussian-based','combined');
    case 'nA'
        legend('n=2','n=3','n=5','n=10');
end
xlabel('time [sec]');
ylabel('entropy [nats]');

% results from planning: RMSE (estimation performance)
figure(12),
plot(sim(1,1).clock.hist.time,rmse); hold on;
switch option
    case 'planner'
        legend('random','w/o comm-aware','disjoint','Gaussian-based','combined');
    case 'nA'
        legend('n=2','n=3','n=5','n=10');
end
xlabel('time [sec]');
ylabel('RMSE [m]');

