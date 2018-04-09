

close all;
clear;
clc;
format compact;
hold on;

%% MONTE-CARLO SETTING %%%%

nSim = 1;

CentralUtil = nan(1,nSim);
DistribUtil = nan(1,nSim);

wCommUtil = nan(1,nSim);
woCommUtil = nan(1,nSim);

for iSim = 1 : nSim
    
    %% INITIAL SETTING %%%%
    
    %--- Simulation Class Setting ----
    nAgent = 1;
    nSeed = iSim;
    
    bPlot = 1;
    bCentral = 1;
    bComm = 1;
    
    mode = 'SDFC';
    % mode = 'MDFC';
    
    if strcmp(mode,'SDFC')
        iDFC = 3;
    else
        iDFC = nan;
    end
    
    SIM = Simulation(nAgent, nSeed, bCentral, bPlot, bComm, mode);
    
    %--- Clock Class Setting ----
    t0 = 0;
    nt = 20; % iteration number
    CLOCK = Clock(t0,nt,SIM);
    
    %--- Environment Classes Setting ----
    ENVIRONMENT = Environment(CLOCK,[-10,10,-10,10],2.5);
    
    %--- Agent Classes Setting ----
    for iAgent = 1 : SIM.nAgent
        AGENT(iAgent) = Agent(ENVIRONMENT, SIM, CLOCK, iAgent, iDFC);
    end
    
    %--- Central Decision-Making Setting ----
    if bCentral == 1
        SIM.DM = CentralDM(SIM,AGENT);
    end
    
    %--- Centralized Optimizer ----
    % SIM.DM.Optimizer(AGENT,ENVIRONMENT,'SDFC');
    
    
    %% MAIN PROCEDURE %%%%
    
    for iClock = 1 : CLOCK.nt
        
        %- Centralized DM --
        if SIM.bCentral == 1
            %--- Compute Action Set ----
            SIM.DM.ComputeAction(0.15, 4, ENVIRONMENT);
            %--- Compute Utility ----
            SIM.DM.ComputeUtility(AGENT,ENVIRONMENT,SIM.mode);
            %--- Take Action ----
            SIM.DM.TakeAction();
        end
        
        %- Distributed DM --
        %--- Compute Action Set ----
        for iAgent = 1 : SIM.nAgent
            AGENT(iAgent).ComputeAction(0.15, 4, ENVIRONMENT);
        end
        
        %--- Compute Utility ----
        for iAgent = 1 : SIM.nAgent
            AGENT(iAgent).ComputeUtility(AGENT,ENVIRONMENT,SIM.mode,SIM.bComm);
        end
        
        %--- Take Action ----
        for iAgent = 1 : SIM.nAgent
            AGENT(iAgent).TakeAction();
        end
        
        %--- Compute Global Utility ----
        SIM.ComputeGlobalUtility(AGENT, ENVIRONMENT, SIM.mode);
        
        %--- clock update ----
        CLOCK.ct = iClock;
        
        %--- plot ----
        if SIM.bPlot == 1
            SIM.Plot(AGENT,ENVIRONMENT,CLOCK);
        end
        
        %--- check the procedure ----
        SIM.CheckTerminateSim(AGENT);
        if SIM.bSim == 0
           break; 
        end
        
        % Create GIF
        frametime = 0.25;
        [imind, cm] = rgb2ind(frame2im(getframe(1)), 256);
        
        if(CLOCK.ct == 1)
            imwrite(imind, cm, 'consensus_iteration.gif', 'LoopCount', inf, 'DelayTime', frametime);
        else
            imwrite(imind, cm, 'consensus_iteration.gif', 'WriteMode', 'append', 'DelayTime', frametime);
        end
        
        
    end

    
    %--- utility profile plot ----
    if SIM.bPlot == 1
        for iAgent = 1 : SIM.nAgent
            figure(2)
            plot(AGENT(iAgent).hist.util,'color',AGENT(iAgent).plot.statecolor,'LineWidth',2); hold on;
            text(CLOCK.ct,AGENT(iAgent).hist.util(end),num2str(AGENT(iAgent).id));
        end
        plot(SIM.hist.util,'--','LineWidth',3);
        if SIM.bCentral == 1
            plot(SIM.DM.hist.util,'.-','LineWidth',3);
        end
    end
    

    %--- algorithm performance data storing ----
    if nSim > 1
        if bCentral == 1
            CentralUtil(iSim) = SIM.DM.hist.util(end);
            DistribUtil(iSim) = SIM.hist.util(end);
        elseif bComm == 1
            wCommUtil(iSim) = SIM.hist.util(end);
        elseif bComm == 0
            woCommUtil(iSim) = SIM.hist.util(end);
        end
        
        fprintf('Sim # = %d\n',iSim)
    end
    
end

if nSim > 1
    if bCentral == 1
        figure(3)
        percent = DistribUtil./CentralUtil;
        hist(percent,0:0.05:1);
        xlabel('W_{distributed}/W_{centralized}','fontsize',20);
        ylabel('Number of occurance','fontsize',20);
        set(gca,'fontsize',20)
        axis([0 1 0 100]);
        
        figure(4)
        h1 = boxplot([DistribUtil',CentralUtil'],{'Game Theoretical','Centralized'});
        ylabel('W','fontsize',20);
        set(gca,'fontsize',20)
    end
end

