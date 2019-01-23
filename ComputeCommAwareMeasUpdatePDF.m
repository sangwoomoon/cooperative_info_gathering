%--------------------------------------------------------------------------------
% equation info
%
% P(x_t|z_t) = [P(z_t|x_t)P(x_t|z_{t-1}}]/[P(z_t|z_{t-1}]   : final prob
% P(z_t|x_t) = P_co(z_t|y_t)P_se(y_t|x_t)                : likelihood function
%
%--------------------------------------------------------------------------------

function [likelihoodPdf,measUpdatePdf] = ...
    ComputeCommAwareMeasUpdatePDF(targetUpdatePdf,pt,plannerAgent,param,meas,commStatus,id,flagComm,flagSensor,flagPdfCompute)

nAgent = length(plannerAgent);

% probability of communication-aware measurement correction P(z_k|X_k):

% 0-1. PDF initialization
measUpdatePdf = targetUpdatePdf;

for iAgent = 1:nAgent
    
    % take likelihood PDF
    likelihoodPdf = ComputeLikelihoodPDF(meas(:,iAgent),pt,plannerAgent(iAgent),param.sensor,param.pdf,flagSensor,flagPdfCompute);
    
    % consider communication awareness
    if flagComm
        % own agent automatically takes measurement update regardless
        % of the communication
        if iAgent == id
            % P(z_t|x_t) = P_co(z_t|y_t)P_se(y_t|x_t)
            measUpdatePdf = measUpdatePdf.*likelihoodPdf;
        else
            beta = ComputeCommProb(plannerAgent(id).s,plannerAgent(iAgent).s);
            
            if commStatus(iAgent) % when connected
                measUpdatePdf = measUpdatePdf.*beta.*likelihoodPdf;
            else % when disconnected
                measUpdatePdf = measUpdatePdf.*(1-beta);
            end
        end
        
    % under perfect communication
    else
        % P(z_t|x_t) = P_co(z_t|y_t)P_se(y_t|x_t)
        measUpdatePdf = measUpdatePdf.*likelihoodPdf;
    end
    
end

% normalization
region = ComputeParticleRegion(pt,param,flagPdfCompute);
normFactor = 1/sum(sum(squeeze(measUpdatePdf).*squeeze(region)));
measUpdatePdf = measUpdatePdf.*normFactor;

end