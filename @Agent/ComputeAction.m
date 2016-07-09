function o = ComputeAction( o, r, n, ENVIRONMENT )
%COMPUTEACTION Summary of this function goes here
%   Detailed explanation goes here
    o.rMove = r;
    o.nMove = n;
    
    ii = 1;
    for iter = 1 : o.nMove
        theta = 2*pi*iter/o.nMove;
        temp = o.s + o.rMove*[cos(theta);sin(theta)];
        if inpolygon(temp(1),temp(2),ENVIRONMENT.bound(:,1),ENVIRONMENT.bound(:,2))
            o.act(ii,:) = temp;
            ii = ii + 1;
        end
        o.act(ii+1,:) = o.s; % stop action
    end

end

