%-----------------------
% Entropy computation:
function entropy = ComputeEntropy(pdf,dRefPt,param)

switch param
    case 'moon'
        nonZeroIdx = pdf > 0; % to prevent from log(0)
        entropy = -sum(pdf(nonZeroIdx).*log(pdf(nonZeroIdx)).*dRefPt);
    case 'ryan'
end

end