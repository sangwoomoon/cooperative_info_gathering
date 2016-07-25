function obj = Initialize( obj, spec, xhat_0, Phat_0, Q )
%INITIALIZEESTIMATOR initialize 
%   Detailed explanation goes here

    xhat = [];
    for iState = 1 : length(xhat_0)
        xhat = [xhat;xhat_0{iState}];
    end

    if length(spec(:,1)) < length(xhat_0) % when it has bias term for estimation    
        obj.bias.x = xhat_0{1};
        obj.bias.Q = Q{1};
        for iTarget = 1 : length(spec(:,1))
            specParam = spec(iTarget);
            TARGET(iTarget) = Target(iTarget,specParam{:});
            TARGET(iTarget).DYNAMICS.Q = Q{iTarget+1};
            TARGET(iTarget).DYNAMICS.x = xhat_0{iTarget+1};
        end
    else
        for iTarget = 1 : length(spec(:,1))
            specParam = spec(iTarget);
            TARGET(iTarget) = Target(iTarget,specParam{:});
            TARGET(iTarget).DYNAMICS.Q = Q{iTarget};
            TARGET(iTarget).DYNAMICS.x = xhat_0{iTarget};
        end
    end
    
    obj.TARGET = TARGET;
    
    obj.nState = length(xhat);
    
    obj.xhat = xhat;
    obj.Phat = Phat_0;
    
    % store this in the history
    obj.hist.xhat = obj.xhat;
    obj.hist.Phat = obj.Phat;
    
    
    

end

