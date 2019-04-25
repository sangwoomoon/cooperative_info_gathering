%--------------------------------------------------------------------------------
% equation info
%
% P(x_t|z_t) = [P(z_t|x_t)P(x_t|z_{t-1}}]/[P(z_t|z_{t-1}]   : final prob
% P(z_t|x_t) = P_co(z_t|y_t)P_se(y_t|x_t)                : likelihood function
%
%--------------------------------------------------------------------------------

function [commProb,measUpdatePdf] = ...
    ComputeCommAwareMeasUpdatePDF(targetUpdatePdf,pt,plannerAgent,param,meas,commStatus,id,flagComm,flagSensor,flagPdfCompute)

nAgent = length(plannerAgent);

% probability of communication-aware measurement correction P(z_k|X_k):

% PDF initialization
measUpdatePdf = targetUpdatePdf;

% beta initialization
commProb = ones(1,nAgent);

% measurement update starts from agent 2 since agent 1 just receives
% information from other agents

% trivial case (ad-hoc implementation): set communication probability of
% its ownship communication situation. This is because of the simulation situation that the agent 1 does not
% take measurement itself.
if commStatus(1) == 1
    commProb(1) = 1;
else
    commProb(1) = 0;
end

for iAgent = 2:nAgent
    
    % considering communication-aware events
    if flagComm
        
        beta = ComputeCommProb(plannerAgent(id).s,plannerAgent(iAgent).s);
        
        if commStatus(iAgent) % when connected
            commProb(iAgent) = beta;
            likelihoodPdf = ComputeLikelihoodPDF(meas(:,iAgent),pt,plannerAgent(iAgent),param.sensor,param.pdf,flagSensor,flagPdfCompute);
            measUpdatePdf = measUpdatePdf.*commProb(iAgent).*likelihoodPdf;
        else % when disconnected
            commProb(iAgent) = 1 - beta;
            measUpdatePdf = measUpdatePdf.*commProb(iAgent);
        end
        
    % when the communication is separatively considered from the measurements
    % : output is real commProb with H(X|Y).
    elseif flagComm == 2
        
        beta = ComputeCommProb(plannerAgent(id).s,plannerAgent(iAgent).s);

        if commStatus(iAgent) % when connected
            commProb(iAgent) = beta;            
        else
            commProb(iAgent) = 1 - beta;            
        end
        
        % take measurement update regardless of the communication status
        likelihoodPdf = ComputeLikelihoodPDF(meas(:,iAgent),pt,plannerAgent(iAgent),param.sensor,param.pdf,flagSensor,flagPdfCompute);
        measUpdatePdf = measUpdatePdf.*likelihoodPdf;        
        
    % under perfect communication
    else
        
        commProb(iAgent) = 1;
        likelihoodPdf = ComputeLikelihoodPDF(meas(:,iAgent),pt,plannerAgent(iAgent),param.sensor,param.pdf,flagSensor,flagPdfCompute);
        measUpdatePdf = measUpdatePdf.*likelihoodPdf;
        
    end

    
end

% normalization
region = ComputeParticleRegion(pt,param,flagPdfCompute);
normFactor = 1/sum(sum(squeeze(measUpdatePdf).*squeeze(region)));
measUpdatePdf = measUpdatePdf.*normFactor;

end