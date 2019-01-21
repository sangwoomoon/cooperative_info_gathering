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
        maxArea = pi*(1*sigma)^2; % maximum area of which radius is 1-sigma
        region = maxArea*ones(1,nPt);
        
        radiusMin = nan(1,nPt);
        for iPt = 1:nPt
            diffX = pt(1,:)-pt(1,iPt);
            diffY = pt(2,:)-pt(2,iPt);
    
            radius = sqrt(diffX.^2 + diffY.^2)/2;
            nonZeroIdx = radius > 0; % to prevent from log(0)
            radiusMin(iPt) = min(radius(nonZeroIdx));
            if sum(nonZeroIdx) == 0
                region(iPt) = pi*0.01^2;
            else
                region(iPt) = pi*(radiusMin(iPt))^2;
            end
        end
        
    case 'voronoi'
        nPt = length(pt(1,:));
        
        sigma = sqrt(param.Q(1,1)+param.Q(2,2));
        maxArea = pi*(1*sigma)^2; % maximum area of which radius is 1-sigma
        region = maxArea*ones(1,nPt);
        
%         % refine domain: in order to remove the same particle location
%         SamePtIdx = zeros(1,nPt);
%         for iPt = 1:nPt
%             SamePtIdx = SamePtIdx + sum(pt == pt(:,iPt))/2;
%         end
        
        % take voronoi tessellation of pdf domain
        [v,c] = voronoin(pt');
        
        % compute area of each voronoi cell w.r.t particles
        for iPt = 1:nPt
            if length(find(c{iPt} - ones(1,length(c{iPt})))) - length(c{iPt}) == 0
                vertPt = v(c{iPt},:);
                region(iPt) = min(polyarea(vertPt(:,1),vertPt(:,2)), region(iPt));
            end
        end
        
        % to check its feasibility (circle assumption)
%         radius = sqrt(region./pi);
%         th = 0:2*pi/20:2*pi;
%         
%         figure(100)
%         plot(pt(1,:),pt(2,:),'m.','linewidth',3); hold on;
%         for iPt = 1:nPt
%             xunit = radius(iPt)*cos(th) + pt(1,iPt);
%             yunit = radius(iPt)*sin(th) + pt(2,iPt);
%             line(xunit,yunit,'color','g');
%         end
%         voronoi(pt(1,:),pt(2,:));
        
end

end