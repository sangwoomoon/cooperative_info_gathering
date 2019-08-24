function PlotPlanningResults(sim,option)

nSim = 9; %length(sim(1,:)); % number of MC simulation runs
mSim = length(sim(:,1)); % number of planning scheme
nPt = sim(1,1).filter(1,1).nPt; % number of particles
nt = sim(1,1).clock.nt;

Hmean = zeros(mSim,nt+1);
rmse = zeros(mSim,nt+1);

% % compute statistical information for comparison between w/ and w/o
% % communication awareness
% 
% % number of agents could be changed by condition
% nAgent = sim(2,1).nAgent;
% nTarget = sim(2,1).nTarget;
% 
% planErr.w = zeros(nAgent,nt+1);
% planErr.wo = zeros(nAgent,nt+1);
% 
% for iSim = 1 : nSim
%     
%     for iAgent = 1 : nAgent
%         
%         % without comm planning
%         I = zeros(size(sim(4,iSim).clock.hist.time));
%         for iTarget = 1 : nTarget
%             I = I + sim(4,iSim).filter(iAgent,iTarget).hist.I;
%         end
%         planErr.wo(iAgent,:) = planErr.wo(iAgent,:) + sim(4,iSim).planner(iAgent).hist.I-I;
%         
%         % with comm planning
%         I = zeros(size(sim(5,iSim).clock.hist.time));
%         for iTarget = 1 : nTarget
%             I = I + sim(5,iSim).filter(iAgent,iTarget).hist.I;
%         end
%         planErr.w(iAgent,:) = planErr.w(iAgent,:) + sim(5,iSim).planner(iAgent).hist.I-I;
%         
%     end
%     % average with respect to simulation runs
%     planErr.w = planErr.w./iSim;
%     planErr.wo = planErr.wo./iSim;
%     
% end


% compute statistical objective values for submodularity and scalablity

% number of agents could be changed by condition
if strcmp(option,'nA')
    objVal = zeros(mSim,nSim);
    
    for jSim = 1 : mSim
        
        % number of agents changed by condition
        nAgent = sim(jSim,1).nAgent;
        
        for iSim = 1 : nSim
            
            for iAgent = 1 : nAgent
                det = sim(jSim,iSim).planner(iAgent).hist.I ~= inf;
                det(1) = 0;
                objValSim = mean(sim(jSim,iSim).planner(iAgent).hist.I(det));
                objVal(jSim,iSim) = objValSim;
            end 
        end
    end
    
    % results from planning: objective function
    h10 = figure(10);
    boxplot(objVal','Labels',{'n_a = 2','n_a = 4','n_a = 6','n_a = 8','n_a = 10'})
    
    set(h10,'Units','Inches');
    pos = get(h10,'Position');
    set(h10,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    set(gca,'FontSize',14);
    print(h10,'utility(n=2,4,6,8,10,t=200,mc=100,RF)','-dpdf','-r0')
    
end


% computate statistical information
for jSim = 1 : mSim

    % number of agents could be changed by condition
    nAgent = sim(jSim,1).nAgent;
    nTarget = sim(jSim,1).nTarget;
    
    for iSim = 1 : nSim
        
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

% % results from planning W/O communication
% figure(10),
% plot(sim(2,1).clock.hist.time,planErr.wo,'r-'); hold on;
% 
% % results from planning W/ communication
% plot(sim(5,1).clock.hist.time,planErr.w,'b-');

% results from planning: entropy variation
h11 = figure(11);
plot(sim(1,1).clock.hist.time,Hmean(1,:),'lineWidth',2,'linestyle',':'); hold on;
plot(sim(1,1).clock.hist.time,Hmean(2,:),'lineWidth',2,'linestyle','--'); hold on;
plot(sim(1,1).clock.hist.time,Hmean(3,:),'lineWidth',2,'linestyle','-.'); hold on;
plot(sim(1,1).clock.hist.time,Hmean(4,:),'lineWidth',2); hold on;
if strcmp(option,'nA')
    plot(sim(1,1).clock.hist.time,Hmean(5,:),'lineWidth',2); hold on;
end
%plot(sim(1,1).clock.hist.time,[Hmean(1:2,:);Hmean(4:5,:)],'lineWidth',2,'fontsize',14); hold on;
%legend('location','southeast');
switch option
    case 'planner'
        legend('random','w/o comm-aware','Gaussian-based','combined');
    case 'nA'
        legend('n=2','n=4','n=6','n=8');
end
xlabel('time [sec]');
ylabel('entropy [nats]');

set(h11,'Units','Inches');
pos = get(h11,'Position');
set(h11,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
set(gca,'FontSize',14);
print(h11,'entropy(n=3,5,10,20,t=200,mc=100,RF)','-dpdf','-r0')

% results from planning: RMSE (estimation performance)
h12 = figure(12);
plot(sim(1,1).clock.hist.time,rmse(1,:),'lineWidth',2,'linestyle',':'); hold on;
plot(sim(1,1).clock.hist.time,rmse(2,:),'lineWidth',2,'linestyle','--'); hold on;
plot(sim(1,1).clock.hist.time,rmse(3,:),'lineWidth',2,'linestyle','-.'); hold on;
plot(sim(1,1).clock.hist.time,rmse(4,:),'lineWidth',2); hold on;
if strcmp(option,'nA')
    plot(sim(1,1).clock.hist.time,rmse(5,:),'lineWidth',2); hold on;
end
switch option
    case 'planner'
        legend('random','w/o comm-aware','Gaussian-based','combined');
    case 'nA'
        legend('n=2','n=4','n=6','n=8','n=10');
end
xlabel('time [sec]');
ylabel('RMSE [m]');

set(h12,'Units','Inches');
pos = get(h12,'Position');
set(h12,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
set(gca,'FontSize',14);
print(h12,'rsme(n=2,4,6,8,10,t=200,mc=100,RF)','-dpdf','-r0')

