
clear;

load('sim_videoclip_RF.mat');
close all;


jSim = 5; % take one condition of simulations (1: random, 2: w/o comm-aware, 3: separate, 4: Gaussian-based 5: combined)
iSim = 1; % take one of simulations
bMovie = 1; % movie making flag (as gif)
bScreen = 0; % save screenshot
fAgent = 1; % agent number to see communication & filtering results

%----------------------
% simulation plotting initialization:
% Figure 1: agent/target moving, comm status & filtering results based on by a picked agent
sim(jSim,iSim).plot.fieldView = figure(1); hold on;    
%----------------------

%----------------------
% plotting setting
figure(1);
set(gca,'xlim',[sim(jSim,iSim).field.boundary(1),sim(jSim,iSim).field.boundary(2)],...
    'ylim',[sim(jSim,iSim).field.boundary(3),sim(jSim,iSim).field.boundary(4)])
set(sim(jSim,iSim).plot.fieldView,'color','w')
xlabel('East [m]'); ylabel('North [m]'); axis equal;
set(gcf,'Position',[500 500 462 420]);
title(sprintf('t = %.1f [sec]', 0))
%----------------------

%----------------------
% filter plotting setting
for iAgent = 1:sim(jSim,iSim).nAgent
    
    for iTarget = 1:sim(jSim,iSim).nTarget
        
        if iAgent == fAgent
            figure(1)
            % actual target plot setting for comparison
            sim(jSim,iSim).filter(iAgent,iTarget).plot.targetPos = ...
                plot(sim(jSim,iSim).target(iTarget).hist.x(1,1),sim(jSim,iSim).target(iTarget).hist.x(2,1),...
                sim(jSim,iSim).target(iTarget).plot.marker,'LineWidth',2,'color',sim(jSim,iSim).target(iTarget).plot.clr);
            sim(jSim,iSim).filter(iAgent,iTarget).plot.targetId = ...
                text(sim(jSim,iSim).target(iTarget).hist.x(1,1),sim(jSim,iSim).target(iTarget).hist.x(2,1),...
                sim(jSim,iSim).plot.targetID(iTarget));
            
            % particle plot
            sim(jSim,iSim).filter(iAgent,iTarget).plot.pt = ...
                scatter(sim(jSim,iSim).filter(iAgent,iTarget).hist.pt(1,:,1),sim(jSim,iSim).filter(iAgent,iTarget).hist.pt(2,:,1),10,sim(jSim,iSim).filter(iAgent,iTarget).hist.w(:,:,1),'filled');
        end
    end
    
end
%----------------------


%----------------------
% agent plotting setting

% agent animation: fixed-wing
scale = 5; % to adjust size of object in fieldView
offset = 30; % to adjust location of agent number in fieldView
fuselargeData = [10 0; 6 -3; -15 0; 6 3; 10 0]'*scale;
mainWingData = [-5 0; 0 15; 1 15; 1 -15; 0 -15; -5 0]'*scale;
tailWingData = [-15 4; -14 4; -13 0; -14 -4; -15 -4]'*scale;

for iAgent = 1:sim(jSim,iSim).nAgent

    % agent position setting
    if iAgent == fAgent
        % change color from bluish to greenish (pick-notation)
        sim(jSim,iSim).agent(iAgent).plot.clr = [0 1 0];
    else
        % change color to blue (for visualization)
        sim(jSim,iSim).agent(iAgent).plot.clr = [0 0 1];
    end
    
    sim(jSim,iSim).agent(iAgent).plot.mainWing = ...
        patch(mainWingData(1,:),mainWingData(2,:),sim(jSim,iSim).agent(iAgent).plot.clr);
    sim(jSim,iSim).agent(iAgent).plot.fuselarge = ...
        patch(fuselargeData(1,:),fuselargeData(2,:),sim(jSim,iSim).agent(iAgent).plot.clr);
    sim(jSim,iSim).agent(iAgent).plot.tailWing = ...
        patch(tailWingData(1,:),tailWingData(2,:),sim(jSim,iSim).agent(iAgent).plot.clr);
    
    % attitude adjusting
    R = [cos(sim(jSim,iSim).agent(iAgent).hist.s(3,1)) -sin(sim(jSim,iSim).agent(iAgent).hist.s(3,1));...
         sin(sim(jSim,iSim).agent(iAgent).hist.s(3,1))  cos(sim(jSim,iSim).agent(iAgent).hist.s(3,1))];
    mainWingDataRot = R*mainWingData;
    fuselargeDataRot = R*fuselargeData;
    tailWingDataRot = R*tailWingData;
    
    % deploy agents
    set(sim(jSim,iSim).agent(iAgent).plot.mainWing,...
        'Xdata',sim(jSim,iSim).agent(iAgent).hist.s(1,1)+mainWingDataRot(1,:),'Ydata',sim(jSim,iSim).agent(iAgent).hist.s(2,1)+mainWingDataRot(2,:),...
        'faceColor',sim(jSim,iSim).agent(iAgent).plot.clr);
    set(sim(jSim,iSim).agent(iAgent).plot.fuselarge,...
        'Xdata',sim(jSim,iSim).agent(iAgent).hist.s(1,1)+fuselargeDataRot(1,:),'Ydata',sim(jSim,iSim).agent(iAgent).hist.s(2,1)+fuselargeDataRot(2,:),...
        'faceColor',sim(jSim,iSim).agent(iAgent).plot.clr);
    set(sim(jSim,iSim).agent(iAgent).plot.tailWing,...
        'Xdata',sim(jSim,iSim).agent(iAgent).hist.s(1,1)+tailWingDataRot(1,:),'Ydata',sim(jSim,iSim).agent(iAgent).hist.s(2,1)+tailWingDataRot(2,:),...
        'faceColor',sim(jSim,iSim).agent(iAgent).plot.clr);
    set(gca,'xlim',[sim(jSim,iSim).field.boundary(1),sim(jSim,iSim).field.boundary(2)],...
        'ylim',[sim(jSim,iSim).field.boundary(3),sim(jSim,iSim).field.boundary(4)])
    sim(jSim,iSim).agent(iAgent).plot.id = ...
        text(sim(jSim,iSim).agent(iAgent).hist.s(1,1)+offset,sim(jSim,iSim).agent(iAgent).hist.s(2,1)+offset,...
        num2str(iAgent));
    
    % agent trajectory plotting
    sim(jSim,iSim).agent(iAgent).plot.path = animatedline(sim(jSim,iSim).agent(iAgent).hist.s(1,1),sim(jSim,iSim).agent(iAgent).hist.s(2,1),'color',sim(jSim,iSim).agent(iAgent).plot.clr);
end
%----------------------

%----------------------
% target plotting setting
for iTarget = 1:sim(jSim,iSim).nTarget
    % position plot setting
    sim(jSim,iSim).target(iTarget).plot.pos = ...
        plot(sim(jSim,iSim).target(iTarget).x(1),sim(jSim,iSim).target(iTarget).x(2),...
        sim(jSim,iSim).target(iTarget).plot.marker,'LineWidth',2,'color',sim(jSim,iSim).target(iTarget).plot.clr);
    sim(jSim,iSim).target(iTarget).plot.id = ...
        text(sim(jSim,iSim).target(iTarget).x(1),sim(jSim,iSim).target(iTarget).x(2),...
        sim(jSim,iSim).plot.targetID(iTarget));
    
    % trajectory plot setting
    sim(jSim,iSim).target(iTarget).plot.path = animatedline(sim(jSim,iSim).target(iTarget).x(1),sim(jSim,iSim).target(iTarget).x(2),'color',sim(jSim,iSim).target(iTarget).plot.clr);
end
%----------------------


%----------------------
% movie initialization
if bMovie
    gif.field.file = ['a',num2str(sim(1).nAgent),'t',num2str(sim(1).nTarget),'_combined_fieldView.gif'];
    gif.field.frame = getframe(sim(jSim,iSim).plot.fieldView);
    gif.field.im = frame2im(gif.field.frame);
    [gif.field.imind,gif.field.cm] = rgb2ind(gif.field.im,256);
    imwrite(gif.field.imind,gif.field.cm,gif.field.file,'gif','DelayTime',0); % 'Loopcount',inf,
    
%     gif.pf.file = ['a',num2str(sim(1).nAgent),'t',num2str(sim(1).nTarget),'_PF.gif'];
%     gif.pf.frame = getframe(sim(jSim,iSim).plot.particle(1));
%     gif.pf.im = frame2im(gif.pf.frame);
%     [gif.pf.imind,gif.pf.cm] = rgb2ind(gif.pf.im,256);
%     imwrite(gif.pf.imind,gif.pf.cm,gif.pf.file,'gif','Loopcount',inf);
end

%----------------------
% save screenshot for initial condition
if bScreen
    set(gcf,'Units','Inches');
    pos = get(gcf,'Position');
    set(gcf,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(gcf,'snapshot_1(t=0,n=4,m=1,RF)','-dpdf','-r0')
end

%% ---------------------------------
% Replay Sim
%-----------------------------------

for iClock = 2:sim(jSim,iSim).clock.nt+1
        
    % particle plot
    for iTarget = 1:sim(jSim,iSim).nTarget
        
        % update plot
        % PICKED AGENT ONLY VISUALIZES PARTICLE INFO BECAUSE OF HUGE
        % PLOTTING SPACE!
        figure(1)
        subplot(sim(jSim,iSim).filter(fAgent,iTarget).plot.location.col,...
            sim(jSim,iSim).filter(fAgent,iTarget).plot.location.row,...
            sim(jSim,iSim).filter(fAgent,iTarget).plot.location.num)
        set(sim(jSim,iSim).filter(fAgent,iTarget).plot.targetPos,'Xdata',sim(jSim,iSim).target(iTarget).hist.x(1,iClock),'Ydata',sim(jSim,iSim).target(iTarget).hist.x(2,iClock));
        set(sim(jSim,iSim).filter(fAgent,iTarget).plot.pt,'Xdata',sim(jSim,iSim).filter(fAgent,iTarget).hist.pt(1,:,iClock),'Ydata',sim(jSim,iSim).filter(fAgent,iTarget).hist.pt(2,:,iClock),...
            'Cdata',sim(jSim,iSim).filter(fAgent,iTarget).hist.w(:,:,iClock));
        set(sim(jSim,iSim).filter(fAgent,iTarget).plot.targetId,'position',...
            [sim(jSim,iSim).target(iTarget).x(1),sim(jSim,iSim).target(iTarget).x(2)]);
        set(gca,'xlim',[sim(jSim,iSim).field.boundary(1),sim(jSim,iSim).field.boundary(2)],...
            'ylim',[sim(jSim,iSim).field.boundary(3),sim(jSim,iSim).field.boundary(4)])
    end
    
    for iAgent = 1:sim(jSim,iSim).nAgent
        
        % agent moving plot
        
        % attitude adjusting
        R = [cos(sim(jSim,iSim).agent(iAgent).hist.s(3,iClock)) -sin(sim(jSim,iSim).agent(iAgent).hist.s(3,iClock));...
            sin(sim(jSim,iSim).agent(iAgent).hist.s(3,iClock))  cos(sim(jSim,iSim).agent(iAgent).hist.s(3,iClock))];
        mainWingDataRot = R*mainWingData;
        fuselargeDataRot = R*fuselargeData;
        tailWingDataRot = R*tailWingData;
        
        if iAgent ~= fAgent
            if sim(jSim,iSim).comm(fAgent).hist.bConnect(iAgent,iClock) == 0
                % non-neighbor representation
                sim(jSim,iSim).agent(iAgent).plot.clr = [1 0 0];
            else
                % neighbor representation
                sim(jSim,iSim).agent(iAgent).plot.clr = [0 0 1];
            end
        end
        
        % agents update
        set(sim(jSim,iSim).agent(iAgent).plot.mainWing,...
            'Xdata',sim(jSim,iSim).agent(iAgent).hist.s(1,iClock)+mainWingDataRot(1,:),'Ydata',sim(jSim,iSim).agent(iAgent).hist.s(2,iClock)+mainWingDataRot(2,:),...
            'faceColor',sim(jSim,iSim).agent(iAgent).plot.clr);
        set(sim(jSim,iSim).agent(iAgent).plot.fuselarge,...
            'Xdata',sim(jSim,iSim).agent(iAgent).hist.s(1,iClock)+fuselargeDataRot(1,:),'Ydata',sim(jSim,iSim).agent(iAgent).hist.s(2,iClock)+fuselargeDataRot(2,:),...
            'faceColor',sim(jSim,iSim).agent(iAgent).plot.clr);
        set(sim(jSim,iSim).agent(iAgent).plot.tailWing,...
            'Xdata',sim(jSim,iSim).agent(iAgent).hist.s(1,iClock)+tailWingDataRot(1,:),'Ydata',sim(jSim,iSim).agent(iAgent).hist.s(2,iClock)+tailWingDataRot(2,:),...
            'faceColor',sim(jSim,iSim).agent(iAgent).plot.clr);
        set(gca,'xlim',[sim(jSim,iSim).field.boundary(1),sim(jSim,iSim).field.boundary(2)],...
            'ylim',[sim(jSim,iSim).field.boundary(3),sim(jSim,iSim).field.boundary(4)])
        set(sim(jSim,iSim).agent(iAgent).plot.id,'position',[sim(jSim,iSim).agent(iAgent).hist.s(1,iClock)+offset,sim(jSim,iSim).agent(iAgent).hist.s(2,iClock)+offset]);
        addpoints(sim(jSim,iSim).agent(iAgent).plot.path,sim(jSim,iSim).agent(iAgent).hist.s(1,iClock),sim(jSim,iSim).agent(iAgent).hist.s(2,iClock));        
    end
    
    % target moving plot
    for iTarget = 1:sim(jSim,iSim).nTarget
        % update plot
        set(sim(jSim,iSim).target(iTarget).plot.pos,'Xdata',sim(jSim,iSim).target(iTarget).hist.x(1,iClock),'Ydata',sim(jSim,iSim).target(iTarget).hist.x(2,iClock));
        set(sim(jSim,iSim).target(iTarget).plot.id,'position',[sim(jSim,iSim).target(iTarget).hist.x(1,iClock),sim(jSim,iSim).target(iTarget).hist.x(2,iClock)]);
        addpoints(sim(jSim,iSim).target(iTarget).plot.path,sim(jSim,iSim).target(iTarget).hist.x(1,iClock),sim(jSim,iSim).target(iTarget).hist.x(2,iClock));
    end
    title(sprintf('t = %.1f [sec]', iClock-1))
        
    
    % draw current states in figures
    drawnow;
    
    % movie file update
    if bMovie
        gif.field.frame = getframe(sim(jSim,iSim).plot.fieldView);
        gif.field.im = frame2im(gif.field.frame);
        [gif.field.imind,gif.field.cm] = rgb2ind(gif.field.im,256);
        imwrite(gif.field.imind,gif.field.cm,gif.field.file,'gif','WriteMode','append','DelayTime',0);
        
%         gif.pf.frame = getframe(sim(jSim,iSim).plot.particle(1));
%         gif.pf.im = frame2im(gif.pf.frame);
%         [gif.pf.imind,gif.pf.cm] = rgb2ind(gif.pf.im,256);
%         imwrite(gif.pf.imind,gif.pf.cm,gif.pf.file,'gif','WriteMode','append');
    end
    
    % for capturing screenshot
    if bScreen
        if iClock == 71
            % save screenshot at t = 70 sec
            print(gcf,'snapshot_2(t=70,n=4,m=1,RF)','-dpdf','-r0')
        elseif iClock == 141
            % save screenshot at t = 140 sec
            print(gcf,'snapshot_3(t=140,n=4,m=1,RF)','-dpdf','-r0')
        end
    end
    
end

% save screenshot at final time step
print(gcf,'snapshot_4(t=200,n=4,m=1,RF)','-dpdf','-r0')
