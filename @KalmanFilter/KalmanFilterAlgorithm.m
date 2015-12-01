function o = KalmanFilterAlgorithm( o, SIMULATION, AGENT, CLOCK, option )

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
    
%%Prediction
Xbar = o.F*o.Xhat; % no input matrix.
Pbar = o.F*o.Phat*o.F' + o.Gamma*o.Q*o.Gamma';

%%Measurement update
innov = o.Y - o.H*Xbar;
S = o.H*Pbar*o.H' + o.R;
W = Pbar*o.H' / S; % Kalman Gain (den : inversed form of S matrix)

Xhat = Xbar + W*innov;
Phat = (eye(o.nState) - W*o.H)*Pbar*(eye(o.nState) - W*o.H)' + W*o.R*W'; % Joseph form to prevent from numerical issues

%%Store and update
o.hist.Y(:,end+1) = o.Y;
o.hist.Xhat(:,end+1) = Xhat;
o.hist.Phat(:,:,end+1) = Phat;
o.hist.stamp(:,end+1) = CLOCK.ct;
o.Xhat = Xhat;
o.Phat = Phat;


end