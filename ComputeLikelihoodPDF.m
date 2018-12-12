%-------------------------------------------------------
% Binary Distribution-based likelihood PDF Generation: P(Y_k|X_k)
%-------------------------------------------------------
function pdf = ComputeLikelihoodPDF(meas,pt,paramAgent,paramSensor,paramPdf,option)

switch option
    case 'uniform'
        pdf = BinarySensorProb(meas,paramAgent,paramPdf.refPt,paramSensor);
        
    case 'cylinder'
        pdf = BinarySensorProb(meas,paramAgent,pt,paramSensor);
        
end

end