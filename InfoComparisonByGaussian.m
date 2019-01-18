
close all; clear;
load('MIcomparison_test.mat');

MIpt = nan(nSim,sim(1).nAgent,61); %sim(1).clock.nt+1);
MIgauss = MIpt;
MIact = zeros(nSim,sim(1).nAgent,61); %sim(1).clock.nt+1);

for iSim = 1 : nSim
    
    for iAgent = 1 : sim(iSim).nAgent
        
        MIpt(iSim,iAgent,:) = abs(sim(iSim).planner(iAgent).hist.I(1:61));
        MIgauss(iSim,iAgent,:) = abs(sim(iSim).planner(iAgent).hist.IRef(1:61));
        
        % accumulate actual MI
        for iTarget = 1 : sim(iSim).nTarget
            MIact(iSim,iAgent,:) = abs(MIact(iSim,iAgent) + sim(iSim).PF(iAgent,iTarget).hist.I(1:61)+2);
        end

        plot(sim(iSim).clock.hist.time(1:61),squeeze(MIgauss(iSim,1,:)),'color','r','LineWidth',2); hold on;
        plot(sim(iSim).clock.hist.time(1:61),squeeze(MIpt(iSim,1,:)),'b','LineWidth',2); hold on;
        plot(sim(iSim).clock.hist.time(1:61),squeeze(MIact(iSim,1,:)),'color','g','LineWidth',2); hold on;
    end
    
end

figure(1)
xlabel('time [sec]');
ylabel('mutual information');
legend('Gaussian assumption','paticle method','true');


MIptErr = rms(squeeze(MIpt(:,:,2:end))'-squeeze(MIact(:,:,2:end))');
MIgaussErr = rms(squeeze(MIgauss(:,:,2:end))'-squeeze(MIact(:,:,2:end))');

figure(2)
histogram(MIgaussErr,20); hold on;
histogram(MIptErr,20);
xlabel('RMS error of mutual information');
ylabel('counts')
legend('Gaussian assumption','paticle method');