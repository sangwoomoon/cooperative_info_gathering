function o = ExtendedKalmanFilterAlgorithm( o, SIMULATION, AGENT, TARGET, CLOCK, option )

% re-initialize matrix
o.Y = [];
o.H = [];

count = 0;

switch (option)
    case 'central'
        for iAgent = 1 : length(AGENT)
            for iTarget = 1 : SIMULATION.nTarget
                o.Y = [o.Y;AGENT(iAgent).MEASURE(iTarget).y];
            end
        end
    case 'local'
        for iAgent = 1 : SIMULATION.nAgent
           if AGENT.COMM.C(iAgent) == 1 % received data from agent (include agent itself)
              if AGENT.COMM.Z(iAgent).targetID == TARGET.id % measured target is the same as target to estimate
                  
                  count = count + 1;

                  o.Y = [o.Y; AGENT.COMM.Z(iAgent).y];
                      % compute measurement matrix
                  o.ComputeH(AGENT.COMM.Z(iAgent).s,TARGET.x);

              end
           end
        end
end

% initialize matrix
o.F = TARGET.Ft;
o.Gamma = TARGET.Gt;
o.Q = TARGET.Qt;
o.V = [];
o.R = [];

for ii = 1 : count
    o.V = blkdiag(o.V, eye(length(AGENT.MEASURE.y)));
    o.R = blkdiag(o.R, AGENT.MEASURE.Rt);
end
    
%%Time update
Xbar = o.F*o.Xhat;
Pbar = o.F*o.Phat*o.F' + o.Gamma*o.Q*o.Gamma';

if count == 0
    Xhat = Xbar;
    Phat = Pbar;
else
    
    %%Measurement update
    L = Pbar*o.H'*(o.H*Pbar*o.H'+o.V*o.R*o.V')^(-1);
    
    G = [];
    
    %--- Measurement ----
    for iAgent = 1 : SIMULATION.nAgent
        if AGENT.COMM.Z(iAgent).targetID == TARGET.id % only concerned agents that measure this target (we know the location of agents via communication)
            dG = o.TakeMeasurement(Xbar,AGENT.COMM.Z(iAgent).s);
            G = [G;dG];
        end
    end
    
    Xhat = Xbar+ L*(o.Y-G);
    Phat = (eye(o.nState) - L*o.H)*Pbar;
    
end

%%Store and update
o.hist.Xhat(:,end+1) = Xhat;
o.hist.Phat(:,:,end+1) = Phat;
o.hist.stamp(:,end+1) = CLOCK.ct;
o.Xhat = Xhat;
o.Phat = Phat;


end