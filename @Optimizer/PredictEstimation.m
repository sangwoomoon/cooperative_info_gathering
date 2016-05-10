function o = PredictEstimation( o, SIMULATION, AGENT, TARGET, CLOCK, option )

% matrix initialize
o.F = [];
o.Gamma = [];
o.Q = [];
o.H = [];
o.V = [];
o.R = [];

% make matrix for assigned targets only (assumed as homogeneous)
for ii = 1 : AGENT.TA.nSearch
    o.F = blkdiag(o.F, AGENT.Fp);
    o.Gamma = blkdiag(o.Gamma, AGENT.Gamp);
    o.Q = blkdiag(o.Q, TARGET(ii).Qt);
    o.ComputeH(AGENT,TARGET(ii));
    o.V = blkdiag(o.V, eye(length(AGENT.MEASURE(ii).y)));
    o.R = blkdiag(o.R, AGENT.MEASURE(ii).Rt);
end

% measurement initialize
o.Y = [];

% merge measurement data with respect to system structure (only assigned
% targets for local case)
switch (option)
    case 'central'
        for iAgent = 1 : length(AGENT)
            for iTarget = 1 : SIMULATION.nTarget
                o.Y = [o.Y;AGENT(iAgent).MEASURE(iTarget).y];
            end
        end
    case 'local'
        for iTarget = 1 : SIMULATION.nTarget
            % find targets that assigned to this agent
            if AGENT.TA.bTasklist(iTarget) == 1
                o.Y = [o.Y;AGENT.MEASURE(iTarget).y];
            end
        end
    case 'decentral'
        for iTarget = 1 : SIMULATION.nTarget
            for iAgent = 1 : SIMULATION.nAgent
                if AGENT.id == iAgent % if index is the agent itself
                    o.u = [o.u;AGENT.CONTROL.u];
                    o.Y = [o.Y;AGENT.MEASURE(iTarget).y];
                else
                    o.u = [o.u;AGENT.COMM.Z(iAgent).u];
                    o.Y = [o.Y;AGENT.COMM.Z(iAgent).y];
                end
            end
        end
    case 'fDDF'
        for iTarget = 1 : SIMULATION.nTarget
            o.Y = [o.Y; AGENT.MEASURE(iTarget).y];
        end
        o.Xhat = AGENT.FDDF.XhatDDF;
        o.Phat = AGENT.FDDF.PhatDDF;
end
    
% extract estimates and covarience from overall data
Xhat = [];
Phat = [];
for iTarget = 1 : SIMULATION.nTarget
    if AGENT.TA.bTasklist(iTarget) == 1
        Xhat = [Xhat;o.Xhat(length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget)];
        Phat = blkdiag(Phat,o.Phat(length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget,...
                        length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget));
    end
end
nState = length(Xhat);


%%Time update
Xbar = o.F*Xhat;
Pbar = o.F*Phat*o.F' + o.Gamma*o.Q*o.Gamma';

%%Measurement update initilization
L = Pbar*o.H'*(o.H*Pbar*o.H'+o.V*o.R*o.V')^(-1);
G = [];

%%Measurement Update
idx = 1;
for iTarget = 1 : length(TARGET)
    if AGENT.TA.bTasklist(iTarget) == 1
        dG = o.TakeMeasurement(Xbar(length(TARGET(iTarget).x)*(idx-1)+1:length(TARGET(iTarget).x)*idx,1),AGENT);
        G = [G;dG];
        idx = idx + 1;
    end
end

Xhat = Xbar+ L*(o.Y-G);
Phat = (eye(nState) - L*o.H)*Pbar;


%%Store and update
idx = 1;
for iTarget = 1 : SIMULATION.nTarget
    
    if AGENT.TA.bTasklist(iTarget) == 1    
        
        o.hist.Y(length(AGENT.MEASURE(iTarget).y)*(iTarget-1)+1:length(AGENT.MEASURE(iTarget).y)*iTarget,CLOCK.ct+1)...
            = o.Y(length(AGENT.MEASURE(iTarget).y)*(idx-1)+1:length(AGENT.MEASURE(iTarget).y)*idx);
        
        o.hist.Xhat(length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget,CLOCK.ct+1)...
            = Xhat(length(TARGET(idx).x)*(idx-1)+1:length(TARGET(iTarget).x)*idx);
        
        o.hist.Phat(length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget,...
                length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget,CLOCK.ct+1)...
            = Phat(length(TARGET(idx).x)*(idx-1)+1:length(TARGET(idx).x)*idx,...
                length(TARGET(idx).x)*(idx-1)+1:length(TARGET(idx).x)*idx);
        
        o.Xhat(length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget) =...
            Xhat(length(TARGET(idx).x)*(idx-1)+1:length(TARGET(iTarget).x)*idx);
        
        o.Phat(length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget,...
                length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget)...
            = Phat(length(TARGET(idx).x)*(idx-1)+1:length(TARGET(idx).x)*idx,...
                length(TARGET(idx).x)*(idx-1)+1:length(TARGET(idx).x)*idx);
        
        idx = idx + 1;

    else
        
        o.hist.Y(length(AGENT.MEASURE(iTarget).y)*(iTarget-1)+1:length(AGENT.MEASURE(iTarget).y)*iTarget,CLOCK.ct+1)...
            = zeros(length(AGENT.MEASURE(iTarget).y),1);
        
        o.hist.Xhat(length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget,CLOCK.ct+1)...
            = zeros(length(TARGET(idx).x),1);
        
        o.hist.Phat(length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget,...
                length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget,CLOCK.ct+1)...
            = zeros(length(TARGET(idx).x),length(TARGET(idx).x));
        
        o.Xhat(length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget) =...
            zeros(length(TARGET(idx).x),1);
        
        o.Phat(length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget,...
                length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget)...
            = zeros(length(TARGET(idx).x),length(TARGET(idx).x));
    end
    
end

o.hist.stamp(:,end+1) = CLOCK.ct;

end