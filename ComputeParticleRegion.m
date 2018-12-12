%-----------------------------------------
% UNIFORM OPTION
% compute the region of sqaure from the discretized domain.
%
% CYLINDER OPTION
% compute the particle region for approximating entropy from the PDF.
% the radius of region is defined as min(r_max,r), where r is determined as
% the mimimum distance betweeen one particle and other particles.
% r_max is determined by 3*sigma range.
% 
% beware of output. it is area of circle of which radius is described
% above.
%-----------------------------------------
function region = ComputeParticleRegion(pt,param,option)

switch option
    % param: dRefPt
    case 'uniform'
        nGrid = length(param.pdf.refPt(1,:,:));
        
        region = param.pdf.dRefPt^2*ones(nGrid);
        
    % param: Q    
    case 'cylinder'
        nPt = length(pt(1,:));
        
        sigma = sqrt(param.Q(1,1)+param.Q(2,2));
        maxArea = pi*(3*sigma)^2; % maximum area of which radius is 3-sigma
        region = maxArea*ones(1,nPt);
                
        for iPt = 1:nPt
            diffX = pt(1,:)-pt(1,iPt);
            diffY = pt(2,:)-pt(2,iPt);
    
            radius = sqrt(diffX.^2 + diffY.^2)/2;
            nonZeroIdx = radius > 0; % to prevent from log(0)
            region(iPt) = min(radius(nonZeroIdx));
        end
        
end

end