
close all; clear;
load('ptVsInfoNoBase.mat');

for iSim = 1 : nSim
   
    for iAgent = 1 : sim(iSim).nAgent
        MI(iSim,iAgent) = mean(sim(iSim).planner(iAgent).hist.I(sim(iSim).planner(iAgent).hist.I(1:end)>0));
        MIref(iSim,iAgent) = mean(sim(iSim).planner(iAgent).hist.IRef(sim(iSim).planner(iAgent).hist.I(1:end)>0));
    end

    nPt(iSim) = sim(iSim).planner(1).PTset.nPt;
end

figure(1)
for iSim = 1 : nSim
    semilogx(nPt,MI(:,iSim),'color',[0.8,0.8,0.8],'LineWidth',2), grid on; hold on;
    semilogx(nPt,MIref(:,iSim),'color',[0.2,1,1],'LineWidth',2), grid on; hold on;
end
xlabel('number of particles')
ylabel('mutual information of planner')

figure(2)
boxplot(MI)