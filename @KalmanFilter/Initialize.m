function obj = Initialize( obj, bTrackTarget, targetSpec, sensorSpec, xhat_0, Phat_0, Q, R )
%INITIALIZEESTIMATOR initialize 
%   Detailed explanation goes here

    xhat = [];
    for iState = 1 : length(xhat_0)
        xhat = [xhat;xhat_0{iState}];
    end
        
    nAgent = length(xhat_0) - length(targetSpec(1,:)); % positive when it has bias term for estimation
    for iAgent = 1 : nAgent % skip when nAgent = 0 (no agent part (e.g. bias)
        SENSOR{iAgent} = SetSensorClass(sensorSpec{iAgent},bTrackTarget(iAgent,:),xhat_0{iAgent},R{iAgent});
        SENSOR{iAgent}.Q = Q{iAgent};
    end
    
    obj.SENSOR = SENSOR;
    
    TARGET = [];
    idxTarget = 0;
    for iTarget = 1 : length(bTrackTarget(1,:))
        if (sum(bTrackTarget(:,iTarget) == 1) >= 1) % at least one of agents track this target
            idxTarget=idxTarget+1;
            if idxTarget == 1
                TARGET = Target(iTarget,targetSpec{idxTarget});
            else
                TARGET(idxTarget) = Target(iTarget,targetSpec{idxTarget});
            end
            TARGET(idxTarget).DYNAMICS.Q = Q{nAgent+idxTarget};
            TARGET(idxTarget).DYNAMICS.x = xhat_0{nAgent+idxTarget};
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


function SENSOR = SetSensorClass(sensorSpec,bTrackingTarget,bias,R)

    switch (sensorSpec)
        case ('RelCartBias')
            SENSOR = RelCartBiasSensor();
            SENSOR.SetParameters([],bTrackingTarget,bias,R); % Track Object / bTrackingTarget / bias / R
        case ('InertCart')
            SENSOR = InertCartSensor();
            SENSOR.SetParameters(
    end

end