%-------------------------------------------------------
% mean/variance-based Gaussian Distribution generation
%-------------------------------------------------------
function pdf = GenerateGaussianPDF(mean,var,param)

% 1. substract mean from domain
ptDomainDiffX = param.refPt(1,:,:) - mean(1);
ptDomainDiffY = param.refPt(2,:,:) - mean(2);

% 2. compute exponential part
ptDomainExpSuper = -1/2.*(1/var(1,1).*ptDomainDiffX.^2 + 1/var(2,2).*ptDomainDiffY.^2);

% 3. compute remain parts. the results is pdf.
pdf = (1/sqrt(2*pi*det(var))).*exp(ptDomainExpSuper);


end