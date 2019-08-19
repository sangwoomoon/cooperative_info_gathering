
clear;

load('sim_demo.mat');
close all;

jSim = 1; % take one condition of simulations (1: random, 2: mean, 3: MI, 4: MI_comm)
iSim = 1; % take one of simulations
bMovie = 0; % movie making flag (as gif)
fAgent = 1; % agent number to see communication & filtering results

%----------------------
% simulation plotting initialization:
% Figure 1: agent/target moving
sim(jSim,iSim).plot.fieldView = figure(1); hold on;
% Figure 2+ (# of agents): estimation. particle evolution
for iAgent = 1:sim(jSim,iSim).nAgent
    % ONLY AGENT 1 PLOTS ESTIMATION RESULT BECAUSE OF HUGE PLOTTING SPACE!!
    if iAgent == fAgent
        sim(jSim,iSim).plot.particle(iAgent) = figure(1+iAgent); hold on;
    end
end
%----------------------

%----------------------
% field plotting setting
figure(1)
set(gca,'xlim',[sim(jSim,iSim).field.boundary(1),sim(jSim,iSim).field.boundary(2)],...
    'ylim',[sim(jSim,iSim).field.boundary(3),sim(jSim,iSim).field.boundary(4)])
set(sim(jSim,iSim).plot.fieldView,'color','w')
xlabel('East [m]'); ylabel('North [m]'); axis equal;
title(sprintf('t = %.1f [sec]', 0))
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
    set(sim(jSim,iSim).agent(iAgent).plot.mainWing,'Xdata',sim(jSim,iSim).agent(iAgent).hist.s(1,1)+mainWingDataRot(1,:),'Ydata',sim(jSim,iSim).agent(iAgent).hist.s(2,1)+mainWingDataRot(2,:));
    set(sim(jSim,iSim).agent(iAgent).plot.fuselarge,'Xdata',sim(jSim,iSim).agent(iAgent).hist.s(1,1)+fuselargeDataRot(1,:),'Ydata',sim(jSim,iSim).agent(iAgent).hist.s(2,1)+fuselargeDataRot(2,:));
    set(sim(jSim,iSim).agent(iAgent).plot.tailWing,'Xdata',sim(jSim,iSim).agent(iAgent).hist.s(1,1)+tailWingDataRot(1,:),'Ydata',sim(jSim,iSim).agent(iAgent).hist.s(2,1)+tailWingDataRot(2,:));
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
% filter plotting setting
for iAgent = 1:sim(jSim,iSim).nAgent
    
    for iTarget = 1:sim(jSim,iSim).nTarget
        
        % particle scatter plot setting
        % AGENT 1 ONLY VISUALIZES PARTICLE BECAUSE OF HUGE PLOTTING SPACE!
        if iAgent == fAgent
            figure(1+iAgent)
            sim(jSim,iSim).plot.particle(iAgent).Units = 'inches';
            sim(jSim,iSim).plot.particle(iAgent).OuterPosition = [0 0 2*ceil(sim(jSim,iSim).nTarget/2) 2*(sim(jSim,iSim).nTarget<2)+4*(sim(jSim,iSim).nTarget>=2)];

            subplot(sim(jSim,iSim).filter(iAgent,iTarget).plot.location.col,...
                sim(jSim,iSim).filter(iAgent,iTarget).plot.location.row,...
                sim(jSim,iSim).filter(iAgent,iTarget).plot.location.num),
            set(gca,'xlim',[sim(jSim,iSim).field.boundary(1),sim(jSim,iSim).field.boundary(2)],...
                'ylim',[sim(jSim,iSim).field.boundary(3),sim(jSim,iSim).field.boundary(4)])
            set(sim(jSim,iSim).plot.particle(iAgent),'color','w')
            xlabel('East [m]'); ylabel('North [m]'); axis equal; hold on;
            
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
% movie initialization
if bMovie
    gif.field.file = ['a',num2str(sim(1).nAgent),'t',num2str(sim(1).nTarget),'_fieldView.gif'];
    gif.field.frame = getframe(sim(jSim,iSim).plot.fieldView);
    gif.field.im = frame2im(gif.field.frame);
    [gif.field.imind,gif.field.cm] = rgb2ind(gif.field.im,256);
    imwrite(gif.field.imind,gif.field.cm,gif.field.file,'gif','Loopcount',inf);
    
    gif.pf.file = ['a',num2str(sim(1).nAgent),'t',num2str(sim(1).nTarget),'_PF.gif'];
    gif.pf.frame = getframe(sim(jSim,iSim).plot.particle(1));
    gif.pf.im = frame2im(gif.pf.frame);
    [gif.pf.imind,gif.pf.cm] = rgb2ind(gif.pf.im,256);
    imwrite(gif.pf.imind,gif.pf.cm,gif.pf.file,'gif','Loopcount',inf);
end


%% ---------------------------------
% Replay Sim
%-----------------------------------

for iClock = 2:sim(jSim,iSim).clock.nt+1
        
    for iAgent = 1:sim(jSim,iSim).nAgent
        
        % agent moving plot
        figure(1)
        % attitude adjusting
        R = [cos(sim(jSim,iSim).agent(iAgent).hist.s(3,iClock)) -sin(sim(jSim,iSim).agent(iAgent).hist.s(3,iClock));...
            sin(sim(jSim,iSim).agent(iAgent).hist.s(3,iClock))  cos(sim(jSim,iSim).agent(iAgent).hist.s(3,iClock))];
        mainWingDataRot = R*mainWingData;
        fuselargeDataRot = R*fuselargeData;
        tailWingDataRot = R*tailWingData;
        
        % agents update
        set(sim(jSim,iSim).agent(iAgent).plot.mainWing,'Xdata',sim(jSim,iSim).agent(iAgent).hist.s(1,iClock)+mainWingDataRot(1,:),'Ydata',sim(jSim,iSim).agent(iAgent).hist.s(2,iClock)+mainWingDataRot(2,:));
        set(sim(jSim,iSim).agent(iAgent).plot.fuselarge,'Xdata',sim(jSim,iSim).agent(iAgent).hist.s(1,iClock)+fuselargeDataRot(1,:),'Ydata',sim(jSim,iSim).agent(iAgent).hist.s(2,iClock)+fuselargeDataRot(2,:));
        set(sim(jSim,iSim).agent(iAgent).plot.tailWing,'Xdata',sim(jSim,iSim).agent(iAgent).hist.s(1,iClock)+tailWingDataRot(1,:),'Ydata',sim(jSim,iSim).agent(iAgent).hist.s(2,iClock)+tailWingDataRot(2,:));
        set(gca,'xlim',[sim(jSim,iSim).field.boundary(1),sim(jSim,iSim).field.boundary(2)],...
            'ylim',[sim(jSim,iSim).field.boundary(3),sim(jSim,iSim).field.boundary(4)])
        set(sim(jSim,iSim).agent(iAgent).plot.id,'position',[sim(jSim,iSim).agent(iAgent).hist.s(1,iClock)+offset,sim(jSim,iSim).agent(iAgent).hist.s(2,iClock)+offset]);
        addpoints(sim(jSim,iSim).agent(iAgent).plot.path,sim(jSim,iSim).agent(iAgent).hist.s(1,iClock),sim(jSim,iSim).agent(iAgent).hist.s(2,iClock));

        % particle plot
        for iTarget = 1:sim(jSim,iSim).nTarget

            % update plot
            % AGENT 1 ONLY VISUALIZES PARTICLE INFO BECAUSE OF HUGE
            % PLOTTING SPACE!
            if iAgent == fAgent
                figure(1+iAgent)
                subplot(sim(jSim,iSim).filter(iAgent,iTarget).plot.location.col,...
                    sim(jSim,iSim).filter(iAgent,iTarget).plot.location.row,...
                    sim(jSim,iSim).filter(iAgent,iTarget).plot.location.num)
                set(sim(jSim,iSim).filter(iAgent,iTarget).plot.targetPos,'Xdata',sim(jSim,iSim).target(iTarget).hist.x(1,iClock),'Ydata',sim(jSim,iSim).target(iTarget).hist.x(2,iClock));
                set(sim(jSim,iSim).filter(iAgent,iTarget).plot.pt,'Xdata',sim(jSim,iSim).filter(iAgent,iTarget).hist.pt(1,:,iClock),'Ydata',sim(jSim,iSim).filter(iAgent,iTarget).hist.pt(2,:,iClock),...
                    'Cdata',sim(jSim,iSim).filter(iAgent,iTarget).hist.w(:,:,iClock));
                set(sim(jSim,iSim).filter(iAgent,iTarget).plot.targetId,'position',...
                    [sim(jSim,iSim).target(iTarget).x(1),sim(jSim,iSim).target(iTarget).x(2)]);
                set(gca,'xlim',[sim(jSim,iSim).field.boundary(1),sim(jSim,iSim).field.boundary(2)],...
                    'ylim',[sim(jSim,iSim).field.boundary(3),sim(jSim,iSim).field.boundary(4)])
            end
            
        end
        
    end
    
    % target moving plot
    figure(1)
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
        imwrite(gif.field.imind,gif.field.cm,gif.field.file,'gif','WriteMode','append');
        
        gif.pf.frame = getframe(sim(jSim,iSim).plot.particle(1));
        gif.pf.im = frame2im(gif.pf.frame);
        [gif.pf.imind,gif.pf.cm] = rgb2ind(gif.pf.im,256);
        imwrite(gif.pf.imind,gif.pf.cm,gif.pf.file,'gif','WriteMode','append');
    end
    
end
