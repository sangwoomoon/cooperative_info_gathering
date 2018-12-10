% ONLY AGENT 2 TAKES MEASUREMENT IN THE SCENARIO: FOR ACC2019 DRAFT
% the code is only applicable in 2 agents scenario, where agent 1 receives
% information from agent 2 which is dependent to the communication status
function [likelihoodPdf,measUpdatePdf] = ComputeCommAwareMeasUpdatePDF(targetUpdatePdf,agent,meas,commStatus,agentParam,sensorParam,pdfParam,nState,id)

nAgent = length(agentParam);

% probability of communication-aware measurement correction P(z_k|X_k):

% 0-1. PDF initialization
measUpdatePdf = targetUpdatePdf;
% 0-2. Comm-related variables initialization
beta = nan(1,nAgent);
commProb = nan(1,nAgent);

for iAgent = 1:nAgent
    
    % 1. take likelihood PDF: NOW IT IS ONLY FOR AGENT i
    likelihoodPdf = ComputeLikelihoodPDF(meas,agentParam(iAgent),sensorParam,pdfParam,nState);

    % 2. compute probability of delivery for communication part
    beta(iAgent) = ComputeCommProb(agent(id).s,agent(iAgent).s);
    
    if commStatus == 1
        commProb(iAgent) = beta(iAgent);
    else
        commProb(iAgent) = 1-beta(iAgent);
    end
    
    measUpdatePdf = measUpdatePdf.*commProb(iAgent).*likelihoodPdf;
end

% normalization
measUpdatePdf = measUpdatePdf./(sum(sum(measUpdatePdf))*(pdfParam.dRefPt^nState));

end