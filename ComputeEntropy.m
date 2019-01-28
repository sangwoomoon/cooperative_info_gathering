%-----------------------
% Entropy computation
function entropy = ComputeEntropy(pdf,pt,param,option)

% compute domain region(each area with respect to particle(cylinder)/discretized
% domain element(uniform)
region = ComputeParticleRegion(pt,param,option);

nonZeroIdx = pdf > 0; % to prevent from log(0)
% prob = pdf(squeeze(nonZeroIdx)).*region(squeeze(nonZeroIdx));
% entropy = -sum(prob.*log(prob));
entropy = -sum(pdf(nonZeroIdx).*log(pdf(nonZeroIdx)).*region(nonZeroIdx));

end