%--------------
% Sum of prob based on particle and weight
function mixedPdf = ComputePDFMixture(pt,w,param,option)
    
nPt = length(pt(1,:));
nState = length(pt(:,1));

for iPt = 1:nPt
    
    % generate pdf which mean is a particle point
    % USER SHOULD MODIFY THE SWITCH WITH RESPECT TO PDF MODEL
    switch option
        case 'Gaussian'
            onePtPdf = GenerateGaussianPDF(pt(:,iPt),param.Q,param.pdf);
        case 'Binary'
            onePtPdf = GenerateBinaryPDF(pt(:,iPt),param.sensor,param.pdf);
    end
    
    % weight sum to make PDF mixture
    if iPt == 1
        mixedPdf = w(iPt)*onePtPdf;
    else
        mixedPdf = mixedPdf+w(iPt)*onePtPdf;
    end

end

% normalize pdf
mixedPdf = mixedPdf./(sum(mixedPdf)*(param.pdf.dRefPt^nState));

end