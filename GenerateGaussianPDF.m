%-------------------------------------------------------
% mean/variance-based Gaussian Distribution generation
%-------------------------------------------------------
function pdf = GenerateGaussianPDF(mean,var,param)

nState = length(mean);
nRefPt = size(param.refPt);

pdf = nan(nRefPt(1),nRefPt(2));

for iRefpt = 1:nRefPt(1)
    for jRefpt = 1:nRefPt(2)
        
        % to address the index of discretized domain
        if nState == 1
            ptDomain = param.refPt(jRefpt);
        elseif nState == 2
            ptDomain = [param.refPt(iRefpt,jRefpt,1),param.refPt(iRefpt,jRefpt,2)]';
        end
        
        pdf(iRefpt,jRefpt) = (1/sqrt(2*pi*det(var)))*exp(-1/2*(ptDomain-mean)'*var^(-1)*(ptDomain-mean));
    end
end

end