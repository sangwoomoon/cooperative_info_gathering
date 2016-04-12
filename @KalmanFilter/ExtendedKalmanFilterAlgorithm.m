function o = ExtendedKalmanFilterAlgorithm( o, SIMULATION, AGENT, TARGET, CLOCK, option )

o.F = [];
o.Gamma = [];
o.Q = [];
o.H = [];
o.V = [];
o.R = [];

for ii = 1 : length(TARGET)
    o.F = blkdiag(o.F, AGENT.Fp);
    o.Gamma = blkdiag(o.Gamma, AGENT.Gamp);
    o.Q = blkdiag(o.Q, TARGET(ii).Qt);
    o.ComputeH(AGENT,TARGET(ii));
    o.V = blkdiag(o.V, eye(length(AGENT.MEASURE(ii).y)));
    o.R = blkdiag(o.R, AGENT.MEASURE(ii).Rt);
end

o.Y = [];

switch (option)
    case 'central'
        for iAgent = 1 : length(AGENT)
            for iTarget = 1 : SIMULATION.nTarget
                o.Y = [o.Y;AGENT(iAgent).MEASURE(iTarget).y];
            end
        end
    case 'local'
        for iTarget = 1 : SIMULATION.nTarget
            o.Y = [o.Y;AGENT.MEASURE(iTarget).y];
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
    
%%Time update
Xbar = o.F*o.Xhat;
Pbar = o.F*o.Phat*o.F' + o.Gamma*o.Q*o.Gamma';

%%Measurement update
L = Pbar*o.H'*(o.H*Pbar*o.H'+o.V*o.R*o.V')^(-1);

G = [];

%--- Measurement ----
for iTarget = 1 : length(TARGET)
    dG = o.TakeMeasurement(Xbar(length(TARGET(iTarget).x)*(iTarget-1)+1:length(TARGET(iTarget).x)*iTarget,1),AGENT);
    G = [G;dG];
end

Xhat = Xbar+ L*(o.Y-G);
Phat = (eye(o.nState) - L*o.H)*Pbar;

%%Store and update
o.hist.Y(:,end+1) = o.Y;
o.hist.Xhat(:,end+1) = Xhat;
o.hist.Phat(:,:,end+1) = Phat;
o.hist.stamp(:,end+1) = CLOCK.ct;
o.Xhat = Xhat;
o.Phat = Phat;


end