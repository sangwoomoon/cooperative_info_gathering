function o = TakeVoronoi( o, AGENT, ENVIRONMENT, SIMULATION )
%TAKEVORONOI Summary of this function goes here
%   Detailed explanation goes here

x = [];
y = [];

w = [];

for iTarget = 1 : SIMULATION.nTarget
    % collect all estimated targets' locations
    x = [x; AGENT.LOCAL_KF(iTarget).Xhat(1)];
    y = [y; AGENT.LOCAL_KF(iTarget).Xhat(3)];
   
    % compute weight from covariance matrix (trace ver.)
    w = [w; 1/2*log((2*pi*exp(1))^(4)*det(AGENT.LOCAL_KF(iTarget).Phat))];
end

if AGENT.id == 1
    fprintf('w = %d\n',w);
end

w = w./sum(w);

min_wts = min(w); % take standard : minimum value

for iter = 1 : length(w)
   if min_wts == w(iter)
       w_new(iter) = 0;
   else
       w_new(iter) = (w(iter)/(w(iter)+min_wts)-0.5)^2*(x(iter)^2+y(iter)^2);
   end
end

if AGENT.id == 1
    fprintf('w_new = %d\n',w_new);
end

rgx = max(ENVIRONMENT.bound(:,1))-min(ENVIRONMENT.bound(:,1));
rgy = max(ENVIRONMENT.bound(:,2))-min(ENVIRONMENT.bound(:,2));
rg = max(rgx,rgy);
midx = (max(ENVIRONMENT.bound(:,1))+min(ENVIRONMENT.bound(:,1)))/2;
midy = (max(ENVIRONMENT.bound(:,2))+min(ENVIRONMENT.bound(:,2)))/2;

% add 4 additional edges
xA = [x; midx + [0;0;-5*rg;+5*rg]];
yA = [y; midy + [-5*rg;+5*rg;0;0]];


[vi,ci]=o.powerDiagram2([xA,yA], [w_new';ones(4,1)]);



% remove the last 4 cells
C = ci(1:end-4);
V = vi;
% use Polybool to crop the cells
%Polybool for restriction of polygons to domain.

maxX = max(ENVIRONMENT.bound(:,1)); minX = min(ENVIRONMENT.bound(:,1));
maxY = max(ENVIRONMENT.bound(:,2)); minY = min(ENVIRONMENT.bound(:,2));
for ij=1:length(C)
    % thanks to http://www.mathworks.com/matlabcentral/fileexchange/34428-voronoilimit
    Cij = C{ij};
    if (length(Cij) == 0) continue; end;
    
    % first convert the contour coordinate to clockwise order:
    pts = V(Cij,:);
    K = convhull(pts);
    K = K(end-1:-1:1);
    C{ij} = Cij(K);
    X2 = pts(K,1);
    Y2 = pts(K,2);
    
    % if all points are inside the bounding box, then skip it
    if (all((X2 <= maxX) & (X2 >= minX) & (Y2 <= maxY) & (Y2 >= minY))) continue; end;
    
    [xb, yb] = polybool('intersection',ENVIRONMENT.bound(:,1),ENVIRONMENT.bound(:,2),X2,Y2);
    % [xb, yb] = clip_polygons(ENVIRONMENT.bound(:,1),ENVIRONMENT.bound(:,2),X2,Y2);
    % xb = xb'; yb = yb';
    ix=nan(1,length(xb));
    for il=1:length(xb)
        if any(V(:,1)==xb(il)) && any(V(:,2)==yb(il))
            ix1=find(V(:,1)==xb(il));
            ix2=find(V(:,2)==yb(il));
            for ib=1:length(ix1)
                if any(ix1(ib)==ix2)
                    ix(il)=ix1(ib);
                end
            end
            if isnan(ix(il))==1
                lv=length(V);
                V(lv+1,1)=xb(il);
                V(lv+1,2)=yb(il);
                ix(il)=lv+1;
            end
        else
            lv=length(V);
            V(lv+1,1)=xb(il);
            V(lv+1,2)=yb(il);
            ix(il)=lv+1;
        end
    end
    C{ij} = ix;
end

o.vertices = V;
o.cell = C; 

end

