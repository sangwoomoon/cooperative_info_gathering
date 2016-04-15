function o = TakeVoronoi( o, AGENT, ENVIRONMENT, SIMULATION )
%TAKEVORONOI Summary of this function goes here
%   Detailed explanation goes here

x = [];
y = [];

% collect all agent's locations
for iAgent = 1 : SIMULATION.nAgent
   x = [x; AGENT(iAgent).s(1)];
   y = [y; AGENT(iAgent).s(3)];
end

rgx = max(ENVIRONMENT.bound(:,1))-min(ENVIRONMENT.bound(:,1));
rgy = max(ENVIRONMENT.bound(:,2))-min(ENVIRONMENT.bound(:,2));
rg = max(rgx,rgy);
midx = (max(ENVIRONMENT.bound(:,1))+min(ENVIRONMENT.bound(:,1)))/2;
midy = (max(ENVIRONMENT.bound(:,2))+min(ENVIRONMENT.bound(:,2)))/2;

% add 4 additional edges
xA = [x; midx + [0;0;-5*rg;+5*rg]];
yA = [y; midy + [-5*rg;+5*rg;0;0]];

[vi,ci]=voronoin([xA,yA]);

% remove the last 4 cells
C = ci(1:end-4);
V = vi;
% use Polybool to crop the cells
%Polybool for restriction of polygons to domain.

for ij=1:length(C)
    % thanks to http://www.mathworks.com/matlabcentral/fileexchange/34428-voronoilimit
    % first convert the contour coordinate to clockwise order:
    [X2, Y2] = poly2cw(V(C{ij},1),V(C{ij},2));
    [xb, yb] = polybool('intersection',ENVIRONMENT.bound(:,1),ENVIRONMENT.bound(:,2),X2,Y2);
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
    C{ij}=ix;
end

o.vertices = V;
o.cell = C; 

end

