function [likelihoodPdf,measUpdatePdf] = ComputeMeasUpdatePDF(targetUpdatePdf,meas,agentPos,sensorParam,pdfParam,nState)

% probability of measurement correction P(y_k|X_k):
likelihoodPdf = ComputeLikelihoodPDF(meas,agentPos,sensorParam,pdfParam,nState);

measUpdatePdf = likelihoodPdf.*targetUpdatePdf;
measUpdatePdf = measUpdatePdf./(sum(measUpdatePdf)*(pdfParam.dRefPt^nState));

end