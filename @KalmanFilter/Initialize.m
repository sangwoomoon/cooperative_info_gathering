function obj = Initialize( obj, targetSpec, sensorSpec, xhat_0, Phat_0, Q, R )
%INITIALIZEESTIMATOR initialize 
%   Detailed explanation goes here

    xhat = [];
    for iState = 1 : length(xhat_0)
        xhat = [xhat;xhat_0{iState}];
    end
        
    nAgent = length(xhat_0) - length(targetSpec(:,1)); % positive when it has bias term for estimation
    for iAgent = 1 : nAgent % skip when nAgent = 0 (no agent part (e.g. bias)
        SENSOR{iAgent} = SensorClassDeclare(sensorSpec(iAgent,:));
        SENSOR{iAgent}.bias = xhat_0{iAgent};
        SENSOR{iAgent}.Q = Q{iAgent};
        SENSOR{iAgent}.R = R{iAgent};
    end
    
    obj.SENSOR = SENSOR;
    
    for iTarget = 1 : length(targetSpec(:,1))
        TARGET(iTarget) = Target(iTarget,targetSpec(iTarget,:));
        TARGET(iTarget).DYNAMICS.Q = Q{iTarget+nAgent};
        TARGET(iTarget).DYNAMICS.x = xhat_0{iTarget+nAgent};
    end
    
    
    obj.TARGET = TARGET;
    
    obj.nState = length(xhat);
    
    obj.xhat = xhat;
    obj.Phat = Phat_0;
    
    % store this in the history
    obj.hist.xhat = obj.xhat;
    obj.hist.Phat = obj.Phat;
    
    
    

end


function SENSOR = SensorClassDeclare(sensorSpec)

    switch (sensorSpec)
        case ('RelCartBias')
            SENSOR = RelCartBiasSensor();
    end

end