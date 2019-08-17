%-------------------------------------------------------
% Binary Distribution-based likelihood PDF Generation: P(Y_k|X_k)
%-------------------------------------------------------
function pdf = ComputeLikelihoodPDF(meas,pt,paramAgent,paramSensor,paramPdf,flagSensor,option)

switch option
    
    case 'uniform'
        
        switch flagSensor
            case 'PosLinear'
                pdf = LinearSensorProb(meas,paramPdf.refPt,paramSensor);
            case 'range_bear'
                pdf = RangeBearProb(meas,paramAgent,paramPdf.refPt,paramSensor);
            case 'bear'
                pdf = BearProb(meas,paramAgent,paramPdf.refPt,paramSensor);
            case 'detection'
                pdf = BinarySensorProb(meas,paramAgent,paramPdf.refPt,paramSensor);
        end
        
        pdfSizeTemp = size(paramPdf.refPt);
        pdfSize = [1 pdfSizeTemp(2:end)];
        pdf = reshape(pdf,pdfSize);
        
    case 'cylinder'
        
        switch flagSensor
            case 'PosLinear'
                pdf = LinearSensorProb(meas,pt,paramSensor);
            case 'range_bear'
                
            case 'detection'
                pdf = BinarySensorProb(meas,paramAgent,pt,paramSensor);
        end
        
    case 'voronoi'
        
        switch flagSensor
            case 'PosLinear'
                pdf = LinearSensorProb(meas,pt,paramSensor);
            case 'range_bear'
                pdf = RangeBearProb(meas,paramAgent,paramPdf.refPt,paramSensor);                
            case 'detection'
                pdf = BinarySensorProb(meas,paramAgent,pt,paramSensor);
        end
        
end

end