function o = ComputeAction( o, r, n, ENVIRONMENT )
%COMPUTEACTION Summary of this function goes here
%   Detailed explanation goes here
    o.rMove = r;
    o.nMove = n;
    
    for iAgent = 1 : length(o.s(1,:))
        ii = 1;
        for iAction = 1 : o.nMove
            theta = 2*pi*iAction/o.nMove;
            temp = o.s(:,iAgent) + o.rMove*[cos(theta);sin(theta)];
            if inpolygon(temp(1),temp(2),ENVIRONMENT.bound(:,1),ENVIRONMENT.bound(:,2))
                o.act(iAgent,ii,:) = temp;
                ii = ii + 1;
            end
        end
        o.act(iAgent,ii+1,:) = o.s(:,iAgent); % stop action
    end

end

