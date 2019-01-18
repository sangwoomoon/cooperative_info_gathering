
close all; clear;
load('ptVsInfoNoBase.mat');

% Particle Number Vs Info (compared by Gaussian assumption)
for iSim = 1 : nSim
    
    for iTrial = 1 : sim(iSim).nAgent
        MI(iTrial,iSim) = mean(sim(iSim).planner(iTrial).hist.I(sim(iSim).planner(iTrial).hist.I(1:end)>0));
        MIref(iTrial,iSim) = mean(sim(iSim).planner(iTrial).hist.IRef(sim(iSim).planner(iTrial).hist.IRef(1:end)>0));
        if isnan(MIref(iTrial,iSim))
            MIref(iTrial,iSim) = 0;
        end
    end

    nPt(iSim) = sim(iSim).planner(iTrial).PTset.nPt;
    nPtLabel{iSim} = num2str(nPt(iSim));
end

figure(1)
for iTrial = 1 : sim(iSim).nAgent
    semilogx(nPt,MI(iTrial,:),'color',[0.8,1,1],'LineWidth',2), grid on; hold on;
    semilogx(nPt,MIref(iTrial,:),'color',[1,0.8,1],'LineWidth',2)
    semilogx(nPt,mean(MI),'color',[0,0,1],'LineWidth',2)
    semilogx(nPt,mean(MIref),'color',[1,0,0],'LineWidth',2)
end
xlabel('number of particles')
ylabel('predicted mutual information')

figure(2)
boxplot(MI, 'Labels', nPtLabel)
xlabel('number of particles')
ylabel('predicted mutual information')

figure(3)
boxplot(MIref, 'Labels', nPtLabel)
xlabel('number of particles')
ylabel('predicted mutual information')
