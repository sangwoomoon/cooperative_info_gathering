

close all;
clear all;
clc;
format compact;
hold on;

%% MONTE-CARLO SETTING %%%%

nSim = 100;

CentralUtil = nan(1,nSim);
DistribUtil = nan(1,nSim);

wCommUtil = nan(1,nSim);
woCommUtil = nan(1,nSim);

for iSim = 1 : nSim
    
    %% INITIAL SETTING %%%%
    
    %--- Simulation Class Setting ----
    nAgent = 4;
    iDFC = 3;
    nSeed = iSim;
    
    bPlot = 0;
    bCentral = 0;
    bComm = 0;
    
    SIM = Simulation(nAgent, nSeed, bCentral, bPlot, bComm);
    
    %--- Clock Class Setting ----
    t0 = 0;
    nt = 150; % iteration number
    CLOCK = Clock(t0,nt,SIM);
    
    %--- Environment Classes Setting ----
    ENVIRONMENT = Environment(CLOCK,[-10,10,-10,10],5);
    
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
            SIM.DM.ComputeUtility(AGENT,ENVIRONMENT,'SDFC');
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
            AGENT(iAgent).ComputeUtility(AGENT,ENVIRONMENT,'SDFC',SIM.bComm);
        end
        
        %--- Take Action ----
        for iAgent = 1 : SIM.nAgent
            AGENT(iAgent).TakeAction();
        end
        
        %--- Compute Global Utility ----
        SIM.ComputeGlobalUtility(AGENT, ENVIRONMENT, 'SDFC');
        
        %--- clock update ----
        CLOCK.ct = iClock;
        
        %--- plot ----
        if SIM.bPlot == 1
            SIM.Plot(AGENT,CLOCK);
            
            axis([ENVIRONMENT.xlength(1) ENVIRONMENT.xlength(2)...
                ENVIRONMENT.ylength(1) ENVIRONMENT.ylength(2)]);
            axis equal;
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
        h1 = boxplot(DistribUtil,'Distributed');
        ylabel('W','fontsize',20);
        set(gca,'fontsize',20)
    end
end

