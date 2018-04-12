%-------------------------------------------------------
% mean/variance-based Gaussian Distribution generation
%-------------------------------------------------------
function pdf = GenerateGaussianPDF(mean,var,param)

pdf = nan(1,length(param.RefPt));

for iRefpt = 1:length(param.RefPt)
    pdf(iRefpt) = (1/sqrt(2*pi*var))*exp(-(param.RefPt(iRefpt)-mean)^2/(2*var));
end

end