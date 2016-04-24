function o = ShareEstimation( o, AGENT, SIMULATION, CLOCK )
%SHAREESTIMATION Summary of this function goes here
%   Detailed explanation goes here
    for iAgent = 1 : SIMULATION.nAgent
        if AGENT.COMM.C(AGENT.id,iAgent) == 1 % if connected and received estimation
            AGENT.LOCAL_EKF.Xhat = AGENT.LOCAL_EKF.Xhat + AGENT.COMM.Z(iAgent).Xhat;
            AGENT.LOCAL_EKF.Phat = AGENT.LOCAL_EKF.Phat + AGENT.COMM.Z(iAgent).Phat;
        end
    end
    
    AGENT.LOCAL_EKF.hist.Xhat(:,CLOCK.ct+1) = AGENT.LOCAL_EKF.Xhat;
    AGENT.LOCAL_EKF.hist.Phat(:,:,CLOCK.ct+1) = AGENT.LOCAL_EKF.Phat;

end

