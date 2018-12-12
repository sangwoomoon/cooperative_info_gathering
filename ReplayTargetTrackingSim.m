
clear;

load('a5t10.mat');
close all;


iSim = 1; % take one of simulations


%----------------------
% simulation plotting initialization:
% Figure 1: agent/target moving
sim(iSim).plot.fieldView = figure(1); hold on;
% Figure 2+ (# of agents): estimation. particle evolution
for iAgent = 1:sim(iSim).nAgent
    % ONLY AGENT 1 PLOTS ESTIMATION RESULT BECAUSE OF HUGE PLOTTING SPACE!!
    if iAgent == 1
        sim(iSim).plot.particle(iAgent) = figure(1+iAgent); hold on;
    end
end
%----------------------

%----------------------
% field plotting setting
figure(1)
set(gca,'xlim',[sim(iSim).field.boundary(1),sim(iSim).field.boundary(2)],...
    'ylim',[sim(iSim).field.boundary(3),sim(iSim).field.boundary(4)])
set(sim(iSim).plot.fieldView,'color','w')
xlabel('East [m]'); ylabel('North [m]'); axis equal;
%----------------------

%----------------------
% target plotting setting
for iTarget = 1:sim(iSim).nTarget
    % position plot setting
    sim(iSim).target(iTarget).plot.pos = ...
        plot(sim(iSim).target(iTarget).hist.x(1,1),sim(iSim).target(iTarget).hist.x(2,1),...
        sim(iSim).target(iTarget).plot.marker,'LineWidth',2,'color',sim(iSim).target(iTarget).plot.clr);
    sim(iSim).target(iTarget).plot.id = ...
        text(sim(iSim).target(iTarget).hist.x(1,1),sim(iSim).target(iTarget).hist.x(2,1),...
        targetID(iTarget));
    
    % trajectory plot setting
    sim(iSim).target(iTarget).plot.path = animatedline(sim(iSim).target(iTarget).hist.x(1,1),sim(iSim).target(iTarget).hist.x(2,1));
end
%----------------------

%----------------------
% agent plotting setting
for iAgent = 1:sim(iSim).nAgent
    % agent position plotting
    sim(iSim).agent(iAgent).plot.pos = ...
        plot(sim(iSim).agent(iAgent).hist.s(1,1),sim(iSim).agent(iAgent).hist.s(2,1),...
        sim(iSim).agent(iAgent).plot.marker,'LineWidth',2,'color',sim(iSim).agent(iAgent).plot.clr);
    sim(iSim).agent(iAgent).plot.id = ...
        text(sim(iSim).agent(iAgent).hist.s(1,1),sim(iSim).agent(iAgent).hist.s(2,1),...
        num2str(iAgent));
    
    % agent trajectory plotting
    sim(iSim).agent(iAgent).plot.path = animatedline(sim(iSim).agent(iAgent).hist.s(1,1),sim(iSim).agent(iAgent).hist.s(2,1));
end
%----------------------

%----------------------
% sensor plotting setting
for iSensor = 1:sim(iSim).nAgent
    % field of view plot
    [~,~,sim(iSim).sensor(iSensor,1).plot.fov] = ...
        GetCircleData(sim(iSim).agent(iSensor).hist.s(1,1),sim(iSim).agent(iSensor).hist.s(2,1),...
        sim(iSim).sensor(iSensor,iTarget).param.regionRadius,...
        sim(iSim).sensor(iSensor,1).plot.clr.noDetect,...
        sim(iSim).sensor(iSensor,1).plot.clr.opaqueValue); hold on;
end
%----------------------

%----------------------
% filter plotting setting
for iAgent = 1:sim(iSim).nAgent
    
    for iTarget = 1:sim(iSim).nTarget
        
        % particle scatter plot setting
        % AGENT 1 ONLY VISUALIZES PARTICLE BECAUSE OF HUGE PLOTTING SPACE!
        if iAgent == 1
            figure(1+iAgent)
            subplot(sim(iSim).PF(iAgent,iTarget).plot.location.col,...
                sim(iSim).PF(iAgent,iTarget).plot.location.row,...
                sim(iSim).PF(iAgent,iTarget).plot.location.num),
            set(gca,'xlim',[sim(iSim).field.boundary(1),sim(iSim).field.boundary(2)],...
                'ylim',[sim(iSim).field.boundary(3),sim(iSim).field.boundary(4)])
            set(sim(iSim).plot.particle(iAgent),'color','w')
            xlabel('East [m]'); ylabel('North [m]'); axis equal; hold on;
            
            % actual target plot setting for comparison
            sim(iSim).PF(iAgent,iTarget).plot.targetPos = ...
                plot(sim(iSim).target(iTarget).hist.x(1,1),sim(iSim).target(iTarget).hist.x(2,1),...
                sim(iSim).target(iTarget).plot.marker,'LineWidth',2,'color',sim(iSim).target(iTarget).plot.clr);
            sim(iSim).PF(iAgent,iTarget).plot.targetId = ...
                text(sim(iSim).target(iTarget).hist.x(1,1),sim(iSim).target(iTarget).hist.x(2,1),...
                targetID(iTarget));
            
            % particle plot
            sim(iSim).PF(iAgent,iTarget).plot.pt = ...
                scatter(sim(iSim).PF(iAgent,iTarget).hist.pt(1,:,1),sim(iSim).PF(iAgent,iTarget).hist.pt(2,:,1),10,sim(iSim).PF(iAgent,iTarget).w,'filled');
            
        end
    end
    
end
%----------------------



%% ---------------------------------
% Replay Sim
%-----------------------------------

for iClock = 2:sim(iSim).clock.nt
        
    for iAgent = 1:sim(iSim).nAgent
        
        % agent moving plot
        figure(1)
        set(sim(iSim).agent(iAgent).plot.pos,'Xdata',sim(iSim).agent(iAgent).hist.s(1,iClock),'Ydata',sim(iSim).agent(iAgent).hist.s(2,iClock));
        set(sim(iSim).agent(iAgent).plot.id,'position',[sim(iSim).agent(iAgent).hist.s(1,iClock),sim(iSim).agent(iAgent).hist.s(2,iClock)]);
        addpoints(sim(iSim).agent(iAgent).plot.path,sim(iSim).agent(iAgent).hist.s(1,iClock),sim(iSim).agent(iAgent).hist.s(2,iClock));
        set(gca,'xlim',[sim(iSim).field.boundary(1),sim(iSim).field.boundary(2)],...
            'ylim',[sim(iSim).field.boundary(3),sim(iSim).field.boundary(4)])
        
        % sensor region plot
        set(sim(iSim).sensor(iAgent,1).plot.fov,'Xdata',sim(iSim).sensor(iAgent,1).plot.hist.data.x(:,iClock),'Ydata',sim(iSim).sensor(iAgent,1).plot.hist.data.y(:,iClock));
        
        if sim(iSim).sensor(iAgent,1).plot.hist.bDetect(:,iClock) % when the sensor detects at least one of targets
            set(sim(iSim).sensor(iAgent,1).plot.fov,'FaceColor',sim(iSim).sensor(iAgent,1).plot.clr.detect);
        else
            set(sim(iSim).sensor(iAgent,1).plot.fov,'FaceColor',sim(iSim).sensor(iAgent,1).plot.clr.noDetect);
        end

        % particle plot
        for iTarget = 1:sim(iSim).nTarget

            % update plot
            % AGENT 1 ONLY VISUALIZES PARTICLE INFO BECAUSE OF HUGE
            % PLOTTING SPACE!
            if iAgent == 1
                figure(1+iAgent)
                subplot(sim(iSim).PF(iAgent,iTarget).plot.location.col,...
                    sim(iSim).PF(iAgent,iTarget).plot.location.row,...
                    sim(iSim).PF(iAgent,iTarget).plot.location.num)
                set(sim(iSim).PF(iAgent,iTarget).plot.targetPos,'Xdata',sim(iSim).target(iTarget).hist.x(1,iClock),'Ydata',sim(iSim).target(iTarget).hist.x(2,iClock));
                set(sim(iSim).PF(iAgent,iTarget).plot.pt,'Xdata',sim(iSim).PF(iPF,iTarget).hist.pt(1,:,iClock),'Ydata',sim(iSim).PF(iPF,iTarget).hist.pt(2,:,iClock),...
                    'Cdata',sim(iSim).PF(iPF,iTarget).hist.w(:,:,iClock));
                set(sim(iSim).PF(iAgent,iTarget).plot.targetId,'position',...
                    [sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2)]);
                set(gca,'xlim',[sim(iSim).field.boundary(1),sim(iSim).field.boundary(2)],...
                    'ylim',[sim(iSim).field.boundary(3),sim(iSim).field.boundary(4)])
            end
            
        end
        
    end
    
    % target moving plot
    figure(1)
    for iTarget = 1:sim(iSim).nTarget
        % update plot
        set(sim(iSim).target(iTarget).plot.pos,'Xdata',sim(iSim).target(iTarget).hist.x(1,iClock),'Ydata',sim(iSim).target(iTarget).hist.x(2,iClock));
        set(sim(iSim).target(iTarget).plot.id,'position',[sim(iSim).target(iTarget).hist.x(1,iClock),sim(iSim).target(iTarget).hist.x(2,iClock)]);
        addpoints(sim(iSim).target(iTarget).plot.path,sim(iSim).target(iTarget).hist.x(1,iClock),sim(iSim).target(iTarget).hist.x(2,iClock));
    end
    
    
    
    drawnow;
end




%%
%----------------------------
% Sim Result Plot
%----------------------------

% % pick one of simulation set
% rSim = ceil(rand(1)*nSim);
% 
% % aircraft trajectories and estimated target location
% figure(1)
% for iTarget = 1:sim(iSim).nTarget
%     plot(sim(rSim).target(iTarget).hist.x(1,:),sim(rSim).target(iTarget).hist.x(2,:),'r-','LineWidth',2); hold on;
%     plot(sim(rSim).target(iTarget).hist.x(1,1),sim(rSim).target(iTarget).hist.x(2,1),'ro','LineWidth',2); hold on;
%     plot(sim(rSim).target(iTarget).hist.x(1,end),sim(rSim).target(iTarget).hist.x(2,end),'rx','LineWidth',2); hold on;
% end
% 
% for iAgent = 1:sim(iSim).nAgent
%     clr = rand(1,3);
%     plot(sim(rSim).agent(iAgent).hist.s(1,:),sim(rSim).agent(iAgent).hist.s(2,:),'-','LineWidth',2,'color',clr); hold on;
%     plot(sim(rSim).agent(iAgent).hist.s(1,1),sim(rSim).agent(iAgent).hist.s(2,1),'o','LineWidth',2,'color',clr); hold on;
%     plot(sim(rSim).agent(iAgent).hist.s(1,end),sim(rSim).agent(iAgent).hist.s(2,end),'x','LineWidth',2,'color',clr); hold on;
% end
% 
% % only for agent 1's info
% for iTarget = 1:sim(iSim).nTarget
%     clr = rand(1,3);
%     plot(sim(rSim).PF(1,iTarget).hist.xhat(1,:),sim(rSim).PF(1,iTarget).hist.xhat(2,:),'--','LineWidth',2,'color',clr); hold on;
%     plot(sim(rSim).PF(1,iTarget).hist.xhat(1,1),sim(rSim).PF(1,iTarget).hist.xhat(2,1),'o','LineWidth',2,'color',clr); hold on;
%     plot(sim(rSim).PF(1,iTarget).hist.xhat(1,end),sim(rSim).PF(1,iTarget).hist.xhat(2,end),'x','LineWidth',2,'color',clr); hold on;
% end
% 
% 
% xlabel('time [sec]'); ylabel('position [m]');
% 
% title('Target Tracking Trajectory and Estimates');
% % legend('true pos','true pos (start)','true pos (end)',...
% %     'Agent 1 pos','Agent 1 pos (start)','Agent 1 (end)',...
% %     'GS estimated pos','GS estimated pos (start)','GS estimated pos (end)',...
% %     'Agent 2 pos','Agent 2 pos (start)','Agent 2 pos (end)',...
% %     'Agent 2 estimated pos','Agent 2 estimated pos (start)','Agent 2 estimated pos (end)');
% % 
% axis equal; axis(sim(rSim).field.boundary);
% 
% %%
% % snapshots and particles
% figure(2)
% if rem(sim(rSim).clock.nt,sim(rSim).param.plot.dClock) == 0
%     nSnapshot = floor(sim(rSim).clock.nt/sim(rSim).param.plot.dClock)+1; % initial, during(dClock/each)-1, final
% else
%     nSnapshot = floor(sim(rSim).clock.nt/sim(rSim).param.plot.dClock)+2; % initial, during(dClock/each), final
% end
% nCol = 3;
% nRow = ceil(nSnapshot/nCol);
% 
% clr = [1 0 0; 0 0 1; 1 1 0];
% ptClr = [1 0 0; 0 0 1; 1 1 0];
% 
% for iSnapshot = 1:nSnapshot
%     subplot(nRow,nCol,iSnapshot)
%     if iSnapshot == 1
%         SnapTime = 1;
%     elseif iSnapshot == nSnapshot
%         SnapTime = sim(rSim).clock.nt+1;
%     else
%         SnapTime = (iSnapshot-1)*sim(rSim).param.plot.dClock+1;
%     end
%     
%     % target position
%     for iTarget = 1:sim(iSim).nTarget % marked as green
%         plot(sim(rSim).target(iTarget).hist.x(1,1:SnapTime),sim(rSim).target(iTarget).hist.x(2,1:SnapTime),'g-','LineWidth',2); hold on;
%         plot(sim(rSim).target(iTarget).hist.x(1,1),sim(rSim).target(iTarget).hist.x(2,1),'go','LineWidth',2); hold on;
%         plot(sim(rSim).target(iTarget).hist.x(1,SnapTime),sim(rSim).target(iTarget).hist.x(2,SnapTime),'gx','LineWidth',2); hold on;
%     end
%     
%     % agent position
%     for iAgent = 1:sim(iSim).nAgent
%         plot(sim(rSim).agent(iAgent).hist.s(1,1:SnapTime),sim(rSim).agent(iAgent).hist.s(2,1:SnapTime),'-','LineWidth',2,'color',clr(iAgent,:)); hold on;
%         plot(sim(rSim).agent(iAgent).hist.s(1,1),sim(rSim).agent(iAgent).hist.s(2,1),'o','LineWidth',2,'color',clr(iAgent,:)); hold on;
%         plot(sim(rSim).agent(iAgent).hist.s(1,SnapTime),sim(rSim).agent(iAgent).hist.s(2,SnapTime),'x','LineWidth',2,'color',clr(iAgent,:)); hold on;
%         
%         % particle plotting
%         plot(squeeze(sim(rSim).PF(iAgent).hist.pt(1,:,SnapTime)),squeeze(sim(rSim).PF(iAgent).hist.pt(2,:,SnapTime)),'.','LineWidth',2,'color',ptClr(iAgent,:)); hold on;
%     end
%     
%     axis equal; axis(sim(rSim).field.boundary);
%     xlabel(['k = ',num2str((SnapTime-1)*sim(rSim).clock.dt),' sec'],'fontsize',12);
% end
% 
% 
% % utility profile
% figure(3)
% for iAgent = 1:sim(iSim).nAgent
%     plot(sim(rSim).clock.hist.time,sim(rSim).planner(iAgent).hist.I,'color',clr(iAgent,:),'LineWidth',3); hold on;
% end
% xlabel('time [sec]'); ylabel('\Sigma_{t=k+1}^{k+T} I(X_t;{^iZ_t}|{^iZ_{k+1:t}}), [nats]');
% legend('Imperfect communication','Perfect communication');
% % title('Utility Profile');
% 
% 
% % utility profile: with respect to single M.I.
% figure(4)
% [timePlanProfile,timeProfile] = meshgrid(1:sim(rSim).planner(1).param.clock.nT,sim(rSim).clock.hist.time);
% mesh(timePlanProfile,timeProfile,sim(rSim).planner(2).hist.Hbefore'-sim(rSim).planner(2).hist.Hafter');
% surface(timePlanProfile,timeProfile,sim(rSim).planner(1).hist.Hbefore'-sim(rSim).planner(1).hist.Hafter');
% xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('mutual information');
% legend('particle-I(x_t;y_t)','particle-I(x_t;z_t)');
% title('Mutual Information Profile');
% 
% 
% % entropy profile
% figure(5)
% [timePlanProfile,timeProfile] = meshgrid(1:sim(rSim).planner(1).param.clock.nT,sim(rSim).clock.hist.time);
% mesh(timePlanProfile,timeProfile,sim(rSim).planner(2).hist.Hbefore');
% surface(timePlanProfile,timeProfile,sim(rSim).planner(1).hist.Hbefore');
% xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('entropy');
% legend('H[P(x_t|y_{k+1:t-1})]','H[P(x_t|z_{k+1:t-1})]');
% view(3);
% title('Entropy of Prior Probability');
% 
% 
% % entropy profile
% figure(6)
% [timePlanProfile,timeProfile] = meshgrid(1:sim(rSim).planner(2).param.clock.nT,sim(rSim).clock.hist.time);
% mesh(timePlanProfile,timeProfile,sim(rSim).planner(2).hist.Hafter');
% surface(timePlanProfile,timeProfile,sim(rSim).planner(1).hist.Hafter');
% xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('entropy');
% legend('H[P(x_t|y_{k+1:t})]','H[P(x_t|z_{k+1:t})]');
% view(3);
% title('Entropy of Posterior Probability');
% 
% 
% % prior/posterior entropy variation w.r.t one actual time steps
% figure(7)
% [timePlanProfile,timeProfile] = meshgrid(reshape(repmat(1:sim(rSim).planner(2).param.clock.nT,2,1),1,[]),sim(rSim).clock.hist.time);
% for iClock = 1:sim(rSim).clock.nt+1
%     entropyProfileGS(iClock,1:2:2*numel(sim(rSim).planner(1).Hbefore)-1) = sim(rSim).planner(1).hist.Hbefore(:,iClock);
%     entropyProfileGS(iClock,2:2:2*numel(sim(rSim).planner(1).Hbefore)) = sim(rSim).planner(1).hist.Hafter(:,iClock);
%     
%     entropyProfilePerfectComm(iClock,1:2:2*numel(sim(rSim).planner(2).Hbefore)-1) = sim(rSim).planner(2).hist.Hbefore(:,iClock);
%     entropyProfilePerfectComm(iClock,2:2:2*numel(sim(rSim).planner(2).Hbefore)) = sim(rSim).planner(2).hist.Hafter(:,iClock);
% end
% mesh(timePlanProfile,timeProfile,entropyProfileGS);
% surface(timePlanProfile,timeProfile,entropyProfilePerfectComm);
% 
% 
% % entropy profile from real PF estimation
% figure(8)
% for iAgent = 1:sim(iSim).nAgent
%     plot(sim(rSim).clock.hist.time,sim(rSim).PF(iAgent).hist.H,'color',clr(iAgent,:),'LineWidth',3); hold on;
% end
% xlabel('time [sec]'); ylabel('entropy [nats]');
% legend('Imperfect communication','Perfect communication');
% % title('Utility Profile');
% 
% 
% %%
% %----------------------------
% % Monte-Carlo based Information analysis
% %----------------------------
% 
% if nSim > 1 % for multiple simulation results
%     Imc = nan(2,nSim);
%     for iSim = 1:nSim
%         Imc(1,iSim) = sim(iSim).planner(1).Isum;
%         Imc(2,iSim) = sim(iSim).planner(2).Isum;
%         ImcRatio(iSim) = Imc(1,iSim)/Imc(2,iSim);
%     end
%     
%     figure(101);
%     for iAgent=1:sim(1).nAgent
%     histogram(Imc(iAgent,:)',40,'FaceColor',clr(iAgent,:)); hold on;
%     end
%     legend('Imperfect communication','perfect communication');
%     % title('Information Distribution: 100 Sims');
%     xlabel('\Sigma_{k=1}^{20}\Sigma_{t=k+1}^{k+T} I(X_t;{^iZ_t}|{^iZ_{k+1:t-1}}), [nats]','Fontsize',12);
%     ylabel('frequency','Fontsize',12);
%     
%     figure(102);
%     histogram(ImcRatio,40);
%     xlabel('Mutual information ratio','Fontsize',12);
%     ylabel('frequency','Fontsize',12);
% end