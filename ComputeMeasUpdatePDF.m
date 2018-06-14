function [likelihoodPdf,measUpdatePdf] = ComputeMeasUpdatePDF(targetUpdatePdf,meas,agentParam,sensorParam,pdfParam,nState)

nAgent = length(agentParam);

% probability of measurement correction P(y_k|X_k):

% 1. take likelihood PDF: production of all likelihoods of agents
likelihoodPdf = ones(length(pdfParam.refPt(:,1,1)),length(pdfParam.refPt(1,:,1)));
for iAgent = 1: nAgent
    likelihoodPdf = likelihoodPdf.*ComputeLikelihoodPDF(meas(iAgent),agentParam(iAgent),sensorParam,pdfParam,nState);
end

measUpdatePdf = likelihoodPdf.*targetUpdatePdf;
measUpdatePdf = measUpdatePdf./(sum(measUpdatePdf)*(pdfParam.dRefPt^nState));

end