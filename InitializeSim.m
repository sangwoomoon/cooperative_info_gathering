function sim = InitializeSim(nAgent,nTarget,flagDM,flagComm,flagPdfCompute,flagLog,flagPlot, target, agent, sensor, filter)

    sim.nAgent = nAgent;
    sim.nTarget = nTarget;
    
    sim.flagScene = 0; % 0: stationary ground station | 1: moving ground station
    sim.flagInfoCom = 1; % 0: Ryan's approach | 1: Our approach(consider all measurements)
    sim.flagDM = flagDM; % 0: 'random': random decision | 'MI': mutual information-based decision | 'mean': particle mean following
    sim.flagPDF = 0; % 0: no PDF draw | 1: PDF draw
    sim.flagComm = flagComm; % 0: perfect communication | 1: imperfect communication and communication awareness
    sim.flagPdfCompute = flagPdfCompute; % 'uniform': uniformly discretized domain | 'cylinder': cylinder based computation w.r.t particle set
    sim.flagLog = flagLog;
    sim.flagPlot = flagPlot; % flag for the display of trajectories and particles evolution
    
    % store property
    sim.flagAgent = agent;
    sim.flagTarget = target;
    sim.flagSensor = sensor;
    sim.flagFilter = filter;
        
    if ~sim.flagPDF
        sim.flagDisp.before = 0;
        sim.flagDisp.after = 0;
    else
        sim.flagDisp.before = 1;
        sim.flagDisp.after = 1;
    end
    
    sim.param.plot.dClock = 10; % interval of snapshot of particle evolution plotting
    
    % simulation plotting initialization:
    % Figure 1: agent/target moving
    if sim.flagPlot
        sim.plot.fieldView = figure(1); hold on;
        set(sim.plot.fieldView,'color','w');
        % Figure 2+ (# of agents): estimation. particle evolution
        % ONLY AGENT 1 PLOTS ESTIMATION RESULT BECAUSE OF HUGE PLOTTING SPACE!!
        %for iAgent = 1:sim(iSim).nAgent
        iAgent = 1;
        sim.plot.particle(iAgent) = figure(1+iAgent); hold on;
        set(sim.plot.particle(iAgent),'color','w')
        %end
        
        sim.plot.targetID = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';
    end

end