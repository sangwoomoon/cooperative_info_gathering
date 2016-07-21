function obj = Initialize( obj, xhat_0, Phat_0 )
%INITIALIZEESTIMATOR initialize 
%   Detailed explanation goes here


    obj.nState = length(xhat_0);
    
    obj.xhat = xhat_0;
    obj.Phat = Phat_0;
    
    % store this in the history
    obj.hist.xhat = obj.xhat;
    obj.hist.Phat = obj.Phat;
    
    

end

