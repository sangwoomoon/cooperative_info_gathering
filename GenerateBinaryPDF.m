%-------------------------------------------------------
% Binary Distribution generation
%-------------------------------------------------------
function pdf = GenerateBinaryPDF(pt,paramSensor,paramPdf)

nState = length(pt);
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
        
        if IsInCircleBasedRegion(pt,ptDomain,paramSensor.regionRadius)
            pdf(iRefpt,jRefpt) = paramSensor.detectBeta;
        else
            pdf(iRefpt,jRefpt) = 1-paramSensor.detectBeta;
        end
    end
end

end