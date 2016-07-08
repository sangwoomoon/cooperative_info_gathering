

close all;
clear all;
clc;
format compact;
hold on;

%% MONTE-CARLO SETTING %%%%

nSim = 50;

CentralUtil = nan(1,nSim);
DistribUtil = nan(1,nSim);

for iSim = 1 : nSim
    
    %% INITIAL SETTING %%%%
    
    %--- Simulation Class Setting ----
    nAgent = 4;
    iDFC = 3;
    nSeed = iSim;
    
    bPlot = 0;
    
    SIM = Simulation(nAgent, nSeed, bPlot);
    
    %--- Clock Class Setting ----
    t0 = 0;
    nt = 200; % iteration number
    CLOCK = Clock(t0,nt,SIM);
    
    %--- Environment Classes Setting ----
    ENVIRONMENT = Environment(CLOCK,[-10,10,-10,10],2);
    
    %--- Agent Classes Setting ----
    for iAgent = 1 : SIM.nAgent
        AGENT(iAgent) = Agent(ENVIRONMENT, SIM, CLOCK, iAgent, iDFC);
    end
    
    %--- Central Decision-Making Setting ----
    SIM.DM = CentralDM(SIM,AGENT);
    
    %% MAIN PROCEDURE %%%%
    
    for iClock = 1 : CLOCK.nt
        
        %- Centralized DM --
        %--- Compute Action Set ----
        SIM.DM.ComputeAction(0.1, 4, ENVIRONMENT);
        %--- Compute Utility ----
        SIM.DM.ComputeUtility(AGENT,ENVIRONMENT,'SDFC');
        %--- Take Action ----
        SIM.DM.TakeAction();
        
        %- Distributed DM --
        %--- Compute Action Set ----
        for iAgent = 1 : SIM.nAgent
            AGENT(iAgent).ComputeAction(0.1, 4, ENVIRONMENT);
        end
        
        %--- Compute Utility ----
        for iAgent = 1 : SIM.nAgent
            AGENT(iAgent).ComputeUtility(AGENT,ENVIRONMENT,'SDFC');
            % AGENT(iAgent).TakeAction();
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
        end
        
    end
    
    
    %--- utility profile plot ----
    if SIM.bPlot == 1
        for iAgent = 1 : SIM.nAgent
            figure(2)
            plot(AGENT(iAgent).hist.util,'color',AGENT(iAgent).plot.statecolor,'LineWidth',2); hold on;
            text(CLOCK.ct,AGENT(iAgent).hist.util(end),num2str(AGENT(iAgent).id));
            plot(SIM.hist.util,'--','LineWidth',3);
            plot(SIM.DM.hist.util,'.-','LineWidth',3);
        end
    end
    
    CentralUtil(iSim) = SIM.DM.hist.util(end);
    DistribUtil(iSim) = SIM.hist.util(end);
    
    fprintf('Sim # = %d\n',iSim)
end

if nSim > 1
   figure(3)
   percent = DistribUtil./CentralUtil;
   hist(percent);
end

