% ONLY AGENT 2 TAKES MEASUREMENT IN THE SCENARIO: FOR ACC2019 DRAFT
% the code is only applicable in 2 agents scenario, where agent 1 receives
% information from agent 2 which is dependent to the communication status
function [likelihoodPdf,measUpdatePdf] = ...
    ComputeCommAwareMeasUpdatePDF(targetUpdatePdf,pt,plannerAgent,param,meas,commStatus,id,flagComm,flagSensor,flagPdfCompute)

nAgent = length(plannerAgent);

% probability of communication-aware measurement correction P(z_k|X_k):

% 0-1. PDF initialization
measUpdatePdf = targetUpdatePdf;
% 0-2. Comm-related variables initialization
beta = nan(1,nAgent);
commProb = nan(1,nAgent);

for iAgent = 1:1 % nAgent
    
    % 1. take likelihood PDF: should be further considered w.r.t
    % measurements!
    likelihoodPdf = ComputeLikelihoodPDF(meas,pt,plannerAgent(iAgent),param.sensor,param.pdf,flagSensor,flagPdfCompute);

    % 2. compute probability of delivery for communication part
    if flagComm == 1
        beta(iAgent) = ComputeCommProb(plannerAgent(id).s,plannerAgent(iAgent).s);
    else
        beta(iAgent) = 1;
    end
    
    if commStatus == 1
        commProb(iAgent) = beta(iAgent);
    else
        commProb(iAgent) = 1-beta(iAgent);
    end
    
    measUpdatePdf = measUpdatePdf.*commProb(iAgent).*likelihoodPdf;
end

% normalization
region = ComputeParticleRegion(pt,param,flagPdfCompute);
measUpdatePdf = measUpdatePdf./(sum(sum(measUpdatePdf))*(sum(sum(region))/numel(measUpdatePdf)));

end