%--------------
% Sum of prob based on particle and weight
function mixedPdf = ComputePDFMixture(pt,w,param,option)
    
nPt = length(pt(1,:));
% nState = length(pt(:,1));

for iPt = 1:nPt
    
    % generate pdf which mean is a particle point
    % USER SHOULD MODIFY THE SWITCH WITH RESPECT TO PDF MODEL
    switch option
        case 'uniform'
            onePtPdf = GenerateGaussianPDF(pt(:,iPt),param.Q,param.pdf);
        case 'cylinder'
            onePtPdf = GenerateGaussianParticlePDF(pt(:,iPt),param.Q,pt);
    end
    
    % weight sum to make PDF mixture
    if iPt == 1
        mixedPdf = w(iPt)*onePtPdf;
    else
        mixedPdf = mixedPdf+w(iPt)*onePtPdf;
    end

end

% normalize pdf for Target GMM
region = ComputeParticleRegion(pt,param,option);
mixedPdf = mixedPdf./(sum(sum(mixedPdf))*(sum(sum(region))/numel(mixedPdf)));

end