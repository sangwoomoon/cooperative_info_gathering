%-------------------------------------------------------
% Binary Distribution-based likelihood PDF Generation: P(Y_k|X_k)
%-------------------------------------------------------
function pdf = ComputeLikelihoodPDF(meas,paramAgent,paramSensor,paramPdf,nState)

nRefPt = size(paramPdf.refPt);

pdf = nan(nRefPt(1),nRefPt(2));

for iRefpt = 1:nRefPt(1)
    for jRefpt = 1:nRefPt(2)
        
        % to address the index of discretized domain
        if nState == 1
            ptDomain = paramPdf.refPt(jRefpt);
        elseif nState == 2
            ptDomain = [paramPdf.refPt(iRefpt,jRefpt,1),paramPdf.refPt(iRefpt,jRefpt,2)]';
        end
        
        pdf(iRefpt,jRefpt) = BinarySensorProb(meas,paramAgent,ptDomain,paramSensor);
    end
end

end