%------------------------------------------------------------------------
% mean/variance-based Gaussian Distribution generation BASED ON PARTICLES
%------------------------------------------------------------------------
function pdf = GenerateGaussianParticlePDF(mean,var,pt)

% 1. substract mean from domain
ptDomainDiffX = pt(1,:) - mean(1);
ptDomainDiffY = pt(2,:) - mean(2);

% 2. compute exponential part
ptDomainExpSuper = -1/2.*(1/var(1,1).*ptDomainDiffX.^2 + 1/var(2,2).*ptDomainDiffY.^2);

% 3. compute remain parts. the results is pdf.
pdf = (1/sqrt(2*pi*det(var))).*exp(ptDomainExpSuper);


end