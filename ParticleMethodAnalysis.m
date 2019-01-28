
% Particle Method Analysis Script
%
%
%
% X(t+1) ~ P_ta(X(t+1)|X(t))
% Y(t)   ~ P_se(Y(t)|X(t);s(t))
% Z(t)   ~ P_co(
%
% - coded by Sangwoo Moon
% - Created:      4/ 3/2018
% - 1st revision: 4/18/2018
% - 2nd revision: 6/25/2018
% - 3rd revision: 9/ 5/2018
% - 4th revision:12/ 8/2018
% - 5th revision: 1/17/2019
%
%   X(t+1) = X(t) + W                               : 2D-static Linear Gaussian
%   Y(t) = {0,1} with respect to Sensing Region     : 2D on the ground, circle region for target detection
%   s(t+1) = f(s(t),u(t))                           : u(t) = [0 -omega +omega]


close all;
clear;
% clc;
format compact;
hold on;

nSim = 1; % for Monte-Carlo approach with fixed independent condition
nPt = [100 500 1000 2000];
dist = [0 200 400 600];
nT = [1 2 3 4 5];

flagCondition  = 'nT';

% simulation by changing independent condition
switch flagCondition
    case 'nPt'
        mSim = length(nPt);
    case 'nT'
        mSim = length(nT);
    case 'dist'
        mSim = length(dist);
    otherwise
        mSim = 1;
end

RandSeed = rng;

% make array of simulation structure
for jSim = 1:mSim
    for iSim = 1:nSim
        
        %-------------------------------
        %   sim setting
        %-------------------------------
        
        rng(RandSeed);
        
        %----------------------
        % simulation structure
        % in order to allocate as the array of simulation
        sim(jSim,iSim) = InitializeSim(   2,       1,     'MI',       1,       'uniform',        0,         0,     'Pos',  'unicycle', 'PosLinear',   'KF'    );
                                     % nAgent | nTarget | flagDM | flagComm | flagPdfCompute | flagLog | flagPlot | target |  agent     | sensor   | filter
        
        % flagDM         ||   'random': random decision | 'MI': mutual information-based decision | 'mean': particle mean following
        % flagComm       ||   0: perfect communication | 1: imperfect communication and communication awareness
        % flagPdfCompute ||   'uniform': uniformly discretized domain | 'cylinder': cylinder based computation w.r.t particle set
        % flagLog        ||   0: skip logging | 1: log data
        % flagPlot       ||   flag for the display of trajectories and particles evolution
        % target         ||   'Pos': position only | 'PosVel': position and velocity
        % sensor         ||   'linear', 'range_bear', 'detection'
        %----------------------
        
    end
end
    

for jSim = 1:mSim
    
    switch flagCondition
        case 'dist'
            % with respect to distance between agents
            fprintf('\njSim = %d, distance = %d\n',jSim,dist(jSim));
        case 'nT'
            % with respect to nT
            fprintf('\njSim = %d, nPt = %d\n',jSim,nT(jSim));
        case 'nPt'
            % with respect to nPt
            fprintf('\njSim = %d, nPt = %d\n',jSim,nPt(jSim));
    end
    
    for iSim = 1:nSim
        
        %----------------------
        % clock structure
        sim(jSim,iSim).clock = InitializeClock(   1  ,   1  );
                                               % nt  |  dt
        %----------------------
        
        %----------------------
        % field structure
        sim(jSim,iSim).field = InitializeField(sim(jSim,iSim), [-300 300],[-300 300]); % overloaded by the number of boundaries (2D)
        % sim(jSim,iSim).field = InitializeField(sim(jSim,iSim), [-300 300],[-300 300],[-300,300]); % overloaded by the number of boundaries (3D)
        %----------------------
        
        %----------------------
        % target structure
        for iTarget = 1:sim(jSim,iSim).nTarget
            sim(jSim,iSim).target(iTarget) = InitializeTarget(iTarget, sim(jSim,iSim));
        end
        %----------------------
        
        %----------------------
        % agent structure
        
        % parameters fleet of agent for initial positioning
        for iAgent = 1:sim(jSim,iSim).nAgent
            sim(jSim,iSim).agent(iAgent) = InitializeAgent(iAgent, sim(jSim,iSim), 10);
        end
        %----------------------
        
        %----------------------
        % sensor structure
        for iAgent = 1:sim(jSim,iSim).nAgent
            for iTarget = 1:sim(jSim,iSim).nTarget
                sim(jSim,iSim).sensor(iAgent,iTarget) = ...
                    InitializeSensor(sim(jSim,iSim),iAgent,iTarget,   40,    0.9,  sim(jSim,iSim).agent(iAgent), sim(jSim,iSim).target(iTarget), diag([20^2,20^2,20^2]'), diag([5^2,(pi/18)^2]') );
                                                                    % range | beta |                                                                      R              |       R_rangebear
            end
        end
        %----------------------
        
        %----------------------
        % communication structure
        for iAgent = 1:sim(jSim,iSim).nAgent
            sim(jSim,iSim).comm(iAgent) = InitializeCommunication(iAgent,sim);
        end
        %----------------------
        
        %----------------------
        % filter structure
        for iAgent = 1:sim(jSim,iSim).nAgent
            for iTarget = 1:sim(jSim,iSim).nTarget
                % nPt = floor(10^(0.15*(iSim+6)));
                xhat = zeros(length(sim(jSim,iSim).target(iTarget).x),1);
                Phat = diag([50^2,50^2,50^2]);
                
                switch flagCondition
                    case 'nPt'
                        sim(jSim,iSim).filter(iAgent,iTarget) = InitializeFilter(sim(jSim,iSim),iAgent,iTarget,  xhat,  Phat,   diag([18^2,18^2,18^2]), nPt(jSim));
                                                                                                               %  xhat | Phat   |            Q         | nPt
                    otherwise
                        sim(jSim,iSim).filter(iAgent,iTarget) = InitializeFilter(sim(jSim,iSim),iAgent,iTarget,  xhat,  Phat,   diag([18^2,18^2,18^2]), nPt(1));
                                                                                                               %  xhat | Phat   |            Q         | nPt
                end
            end
        end
        %----------------------
        
        %----------------------
        % planner structure
        for iAgent = 1:sim(jSim,iSim).nAgent
            
            switch flagCondition
                case 'nPt'
                    sim(jSim,iSim).planner(iAgent) = InitializePlanner(iAgent,sim(jSim,iSim), 3,  nT(3),  nPt(jSim) );
                                                                                            % dt | nT |     nPt
                otherwise
                    switch flagCondition
                        case 'nT'
                            sim(jSim,iSim).planner(iAgent) = InitializePlanner(iAgent,sim(jSim,iSim), 3,  nT(jSim),  nPt(1) );
                                                                                                   % dt |     nT   | nPt
                        otherwise
                            sim(jSim,iSim).planner(iAgent) = InitializePlanner(iAgent,sim(jSim,iSim), 3,  nT(3),  nPt(1) );
                                                                                                   % dt |   nT |    nPt
                    end
            end
        end
        %----------------------
        
        
        %% ---------------------------------
        % Sim Operation
        %-----------------------------------
        
        for iClock = 1:sim(jSim,iSim).clock.nt
            
            %-----------------------------------
            % PF-based Mutual information Computation and decision-making
            %-----------------------------------
            
            % compute future information with respect to action profiles
            % distributed scheme to each agent:
            % COMPUTED INFORMATION IS DIFFERENT WITH RESPECT TO AGENT
            for iAgent = 1:sim(jSim,iSim).nAgent
                
                iAction = 1; % this script does not consider action candidate: only one action profile
                %---------------------------------------------------------------------------------------------------------
                % Mutual Information computation:
                %
                % FIVE APPROACHES are implemented
                %
                % 1. particle method considers all measurement/communication awareness possibilities.
                % 2. particle method of which communication is sampled by Pco(Z|Y): motivated by Ryan's approach
                % 3. Gaussian approximation of which communication is sampled by Pco(Z|Y): motivated by Ryan's approach
                % 4. Gaussian approximation with modified covariance approach: Maicej's approach
                % 5. Gaussian with all measurement/communication possibilities: exact when the model is Linear/Gaussian
                %
                if sim(jSim,iSim).flagComm
                    [pmAll, pmSample, pmSeparate, gaussRtilde, gaussAll] = ComputeInformation(iAgent,iAction,iClock,sim(jSim,iSim));
                else
                    [pmAll, ~, ~, ~, gaussAll] = ComputeInformation(iAgent,iAction,iClock,sim(jSim,iSim));
                end
                %---------------------------------------------------------------------------------------------------------
                
                % store information data into
                sim(jSim,iSim).planner(iAgent).pmAll = pmAll;
                sim(jSim,iSim).planner(iAgent).gaussAll = gaussAll;
                
                if sim(jSim,iSim).flagComm
                    sim(jSim,iSim).planner(iAgent).pmSample = pmSample;
                    sim(jSim,iSim).planner(iAgent).pmSeparate = pmSeparate;
                    sim(jSim,iSim).planner(iAgent).gaussRtilde = gaussRtilde;
                end
                
                % plot the information profile when the simulation does not
                % take Monte-Carlo process
                if nSim == 1
                    PlotInformation(sim(jSim,iSim).planner(iAgent),sim(jSim,iSim).flagSensor,sim(jSim,iSim).flagComm,jSim);
                end
                
            end
            
            %         %-----------------------------------
            %         % Actual Agent-Target Dynamics and Measurement
            %         %-----------------------------------
            %
            %         %-----------------------------------
            %         % target moving
            %         for iTarget = 1:sim(iSim).nTarget
            %             % target dynamics/store data
            %             sim(iSim).target(iTarget).x = UpdateTargetState(sim(iSim).target(iTarget).x,sim(iSim).target(iTarget).param,sim(iSim).clock.dt);
            %             sim(iSim).target(iTarget).hist.x(:,iClock+1) = sim(iSim).target(iTarget).x;
            %
            %             % update plot
            %             if sim(iSim).flagPlot
            %                 set(sim(iSim).target(iTarget).plot.pos,'Xdata',sim(iSim).target(iTarget).x(1),'Ydata',sim(iSim).target(iTarget).x(2));
            %                 set(sim(iSim).target(iTarget).plot.id,'position',[sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2)]);
            %                 addpoints(sim(iSim).target(iTarget).plot.path,sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2));
            %             end
            %         end
            %         %-----------------------------------
            %
            %         for iAgent = 1:sim(iSim).nAgent
            %
            %             %-----------------------------------
            %             % agent moving
            %             % agent dynamics/store data
            %             sim(iSim).agent(iAgent).s = UpdateAgentState(sim(iSim).agent(iAgent).s,sim(iSim).planner(iAgent).input(1),sim(iSim).clock.dt);
            %             sim(iSim).agent(iAgent).hist.s(:,iClock+1) = sim(iSim).agent(iAgent).s;
            %
            %             % update plot
            %             if sim(iSim).flagPlot
            %                 set(sim(iSim).agent(iAgent).plot.pos,'Xdata',sim(iSim).agent(iAgent).s(1),'Ydata',sim(iSim).agent(iAgent).s(2));
            %                 set(sim(iSim).agent(iAgent).plot.id,'position',[sim(iSim).agent(iAgent).s(1),sim(iSim).agent(iAgent).s(2)]);
            %                 addpoints(sim(iSim).agent(iAgent).plot.path,sim(iSim).agent(iAgent).s(1),sim(iSim).agent(iAgent).s(2));
            %                 set(gca,'xlim',[sim(iSim).field.boundary(1),sim(iSim).field.boundary(2)],...
            %                     'ylim',[sim(iSim).field.boundary(3),sim(iSim).field.boundary(4)])
            %             end
            %             %-----------------------------------
            %
            %
            %             %-----------------------------------
            %             % take measurement
            %             sim(iSim).sensor(iAgent,1).plot.bDetect = 0;
            %             for iTarget = 1:sim(iSim).nTarget
            %                 sim(iSim).sensor(iAgent,iTarget).y = ...
            %                     TakeMeasurement(sim(iSim).target(iTarget).x,sim(iSim).agent(iAgent).s,sim(iSim).sensor(iAgent,iTarget).param,sim(iSim).flagSensor);
            %                 sim(iSim).sensor(iAgent,iTarget).hist.y(:,iClock+1) = sim(iSim).sensor(iAgent,iTarget).y;
            %
            %                 if sim(iSim).sensor(iAgent,iTarget).y == 1
            %                     sim(iSim).sensor(iAgent,1).plot.bDetect = 1;
            %                 end
            %             end
            %             sim(iSim).sensor(iAgent,1).plot.hist.bDetect(:,iClock+1) = sim(iSim).sensor(iAgent,1).plot.bDetect;
            %
            %             % update plot
            %             if sim(iSim).flagPlot
            %                 [sim(iSim).sensor(iAgent,1).plot.data.x,sim(iSim).sensor(iAgent,1).plot.data.y,~] = ...
            %                     GetCircleData(sim(iSim).agent(iAgent).s(1),sim(iSim).agent(iAgent).s(2),sim(iSim).sensor(iAgent,1).param.regionRadius);
            %                 set(sim(iSim).sensor(iAgent,1).plot.fov,'Xdata',sim(iSim).sensor(iAgent,1).plot.data.x,'Ydata',sim(iSim).sensor(iAgent,1).plot.data.y);
            %                 sim(iSim).sensor(iAgent,1).plot.hist.data.x(:,iClock+1) = sim(iSim).sensor(iAgent,1).plot.data.x';
            %                 sim(iSim).sensor(iAgent,1).plot.hist.data.y(:,iClock+1) = sim(iSim).sensor(iAgent,1).plot.data.y';
            %
            %                 if sim(iSim).sensor(iAgent,1).plot.bDetect % when the sensor detects at least one of targets
            %                     set(sim(iSim).sensor(iAgent,1).plot.fov,'FaceColor',sim(iSim).sensor(iAgent,1).plot.clr.detect);
            %                 else
            %                     set(sim(iSim).sensor(iAgent,1).plot.fov,'FaceColor',sim(iSim).sensor(iAgent,1).plot.clr.noDetect);
            %                 end
            %             end
            %             %-----------------------------------
            %
            %
            %             %-----------------------------------
            %             % particle measurement and agent state sharing through communication
            %             [sim(iSim).comm(iAgent).beta,sim(iSim).comm(iAgent).bConnect,sim(iSim).planner(iAgent).agent,sim(iSim).comm(iAgent).z] = ...
            %                 ShareInformation(sim(iSim).agent,sim(iSim).sensor,sim(iSim).planner(iAgent).agent,sim(iSim).filter(iAgent).id(1), sim(iSim).flagComm);
            %             sim(iSim).comm(iAgent).hist.beta(:,iClock+1) = sim(iSim).comm(iAgent).beta;
            %             sim(iSim).comm(iAgent).hist.bConnect(:,iClock+1) = sim(iSim).comm(iAgent).bConnect;
            %             sim(iSim).comm(iAgent).hist.Z(:,:,iClock+1) = sim(iSim).comm(iAgent).z';
            %             %-----------------------------------
            %
            %             %-----------------------------------
            %             % Actual measurement and estimation: PF
            %             %
            %             % PF is locally performend, and measurement information is delivered
            %             % under the communication probability
            %             for iTarget = 1:sim(iSim).nTarget
            %                 % particle state update
            %                 sim(iSim).filter(iAgent,iTarget).pt = UpdateParticle(sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).filter(iAgent,iTarget).param,sim(iSim).clock.dt);
            %
            %                 % particle weight update
            %                 sim(iSim).filter(iAgent,iTarget).w = UpdateParticleWeight(sim(iSim).comm(iAgent).z(:,iTarget),sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).planner(iAgent).agent,sim(iSim).sensor(iAgent).param);
            %
            %                 % compute actual entropy for comparison
            %                 targetUpdatePdf = ComputePDFMixture(sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).filter(iAgent,iTarget).w,sim(iSim).planner(iAgent).param,sim(iSim).flagPdfCompute);
            %                 sim(iSim).filter(iAgent,iTarget).Hbefore = ComputeEntropy(targetUpdatePdf,sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).planner(iAgent).param,sim(iSim).flagPdfCompute);
            %                 sim(iSim).filter(iAgent,iTarget).hist.Hbefore(:,iClock+1) = sim(iSim).filter(iAgent,iTarget).Hbefore;
            %
            %
            %                 % update plot
            %                 % AGENT 1 ONLY VISUALIZES PARTICLE INFO BECAUSE OF HUGE
            %                 % PLOTTING SPACE!
            %                 if (sim(iSim).flagPlot) && (iAgent == 1)
            %                     figure(1+iAgent)
            %                     subplot(sim(iSim).filter(iAgent,iTarget).plot.location.col,...
            %                         sim(iSim).filter(iAgent,iTarget).plot.location.row,...
            %                         sim(iSim).filter(iAgent,iTarget).plot.location.num)
            %                     set(sim(iSim).filter(iAgent,iTarget).plot.targetPos,'Xdata',sim(iSim).target(iTarget).x(1),'Ydata',sim(iSim).target(iTarget).x(2));
            %                     set(sim(iSim).filter(iAgent,iTarget).plot.pt,'Xdata',sim(iSim).filter(iAgent,iTarget).pt(1,:),'Ydata',sim(iSim).filter(iAgent,iTarget).pt(2,:),...
            %                         'Cdata',sim(iSim).filter(iAgent,iTarget).w);
            %                     set(sim(iSim).filter(iAgent,iTarget).plot.targetId,'position',...
            %                         [sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2)]);
            %                     set(gca,'xlim',[sim(iSim).field.boundary(1),sim(iSim).field.boundary(2)],...
            %                         'ylim',[sim(iSim).field.boundary(3),sim(iSim).field.boundary(4)])
            %                 end
            %
            %
            %                 % resample particle
            %                 [sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).filter(iAgent,iTarget).w] = ResampleParticle(sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).filter(iAgent,iTarget).w,sim(iSim).field);
            %
            %                 % particle filter info update/store
            %                 sim(iSim).filter(iAgent,iTarget).xhat = (sim(iSim).filter(iAgent,iTarget).w*sim(iSim).filter(iAgent,iTarget).pt')';
            %                 sim(iSim).filter(iAgent,iTarget).hist.pt(:,:,iClock+1) = sim(iSim).filter(iAgent,iTarget).pt;
            %                 sim(iSim).filter(iAgent,iTarget).hist.w(:,:,iClock+1) = sim(iSim).filter(iAgent,iTarget).w;
            %                 sim(iSim).filter(iAgent,iTarget).hist.xhat(:,iClock+1) = sim(iSim).filter(iAgent,iTarget).xhat;
            %
            %
            %                 % update planner initial info
            %                 sim(iSim).planner(iAgent).PTset(iTarget).xhat = sim(iSim).filter(iAgent,iTarget).xhat;
            %                 sim(iSim).planner(iAgent).PTset(iTarget).w = sim(iSim).filter(iAgent,iTarget).w;
            %                 sim(iSim).planner(iAgent).PTset(iTarget).pt = sim(iSim).filter(iAgent,iTarget).pt;
            %
            %                 % store optimized infomation data
            %                 sim(iSim).planner(iAgent).hist.actIdx(iClock+1) = sim(iSim).planner(iAgent).actIdx;
            %                 sim(iSim).planner(iAgent).hist.input(:,iClock+1) = sim(iSim).planner(iAgent).input;
            %
            %                 % store entropy-based data for planner: only useful for MI-based planning
            %                 if strcmp(sim(iSim).flagDM,'MI')
            %                     for iAgent = 1:sim(iSim).nAgent
            %                         sim(iSim).planner(iAgent).hist.I(:,iClock+1) = sim(iSim).planner(iAgent).I;
            %                         sim(iSim).planner(iAgent).hist.Hafter(:,iClock+1) = sim(iSim).planner(iAgent).Hafter';
            %                         sim(iSim).planner(iAgent).hist.Hbefore(:,iClock+1) = sim(iSim).planner(iAgent).Hbefore';
            %
            %                         sim(iSim).planner(iAgent).hist.IRef(:,iClock+1) = sim(iSim).planner(iAgent).IRef;
            %                         sim(iSim).planner(iAgent).hist.HafterRef(:,iClock+1) = sim(iSim).planner(iAgent).HafterRef';
            %                         sim(iSim).planner(iAgent).hist.HbeforeRef(:,iClock+1) = sim(iSim).planner(iAgent).HbeforeRef';
            %                     end
            %                 end
            %
            %                 % compute actual entropy for comparison
            %                 targetUpdatePdf = ComputePDFMixture(sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).filter(iAgent,iTarget).w,sim(iSim).planner(iAgent).param,sim(iSim).flagPdfCompute);
            %                 sim(iSim).filter(iAgent,iTarget).Hafter = ComputeEntropy(targetUpdatePdf,sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).planner(iAgent).param,sim(iSim).flagPdfCompute);
            %                 sim(iSim).filter(iAgent,iTarget).hist.Hafter(:,iClock+1) = sim(iSim).filter(iAgent,iTarget).Hafter;
            %
            %                 % compute actual entropy reduction for comparison
            %                 sim(iSim).filter(iAgent,iTarget).I = sim(iSim).filter(iAgent,iTarget).Hbefore - sim(iSim).filter(iAgent,iTarget).Hafter;
            %                 sim(iSim).filter(iAgent,iTarget).hist.I(:,iClock+1) = sim(iSim).filter(iAgent,iTarget).I;
            %
            %             end
            %             %-----------------------------------
            %
            %
            %             %-----------------------------------
            %             % take decision making for agent input
            %             sim(iSim).agent(iAgent).vel = sim(iSim).planner(iAgent).input(1);
            %             %-----------------------------------
            %
            %
            %         end
            
            % clock update
            sim(jSim,iSim).clock.hist.time(:,iClock+1) = iClock*sim(jSim,iSim).clock.dt;
            
            % update figure
            if sim(jSim,iSim).flagPlot
                drawnow;
                figure(1)
                title(sprintf('t = %.1f [sec]', sim(jSim,iSim).clock.hist.time(:,iClock+1)))
            end
            
        end
        
        
        % display current simulation number
        fprintf('iSim = %d\n',iSim);
        
        
    end
    
        fprintf('===========\n');
end



if jSim > 1
    PlotMonteCarloResults(sim,mSim,nSim,flagCondition);    
end
