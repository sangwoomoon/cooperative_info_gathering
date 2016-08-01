function jacobian = GatherJacobian( obj, CLOCK, option, Y_k )
%GATHERJACOBIANMATRIX GroupS Jacobian matrix with respect to targets and bias

% used input : ESTIMATOR.TARGET.DYNAMICS / ESTIMATOR.SENSOR (if bias exists)

jacobian = [];

switch (option)
    
    case ('state') % state transition (grouped Phi)
        
        % when sensor has bias and this should be estimated, then plug this
        % into the groupped jacobian matrix
        % it is odd. should be capsulated in the RelCartBias class!!!
        for iSensor = 1 : length(obj.SENSOR)
            if strcmp(obj.SENSOR{iSensor}.spec,'RelCartBias')
                jacobian = blkdiag(jacobian, obj.SENSOR{iSensor}.TakeBiasJacobian('BiasState'));
            end
        end
        
        for iTarget = 1 : length(obj.TARGET)
            jacobian = blkdiag(jacobian, obj.TARGET(iTarget).DYNAMICS.TakeJacobian(obj.TARGET(iTarget).CONTROL.u, CLOCK.dt, 'state'));
        end
        
    case ('noise') % process noise (grouped Gamma)
        
        % when sensor has bias and this should be estimated, then plug this
        % into the groupped jacobian matrix
        % it is odd. should be capsulated in the RelCartBias class!!!
        for iSensor = 1 : length(obj.SENSOR)
            if strcmp(obj.SENSOR{iSensor}.spec,'RelCartBias')
                jacobian = blkdiag(jacobian, obj.SENSOR{iSensor}.TakeBiasJacobian('BiasNoise'));
            end
        end

        for iTarget = 1 : length(obj.TARGET)
            jacobian = blkdiag(jacobian, obj.TARGET(iTarget).DYNAMICS.TakeJacobian(obj.TARGET(iTarget).CONTROL.u, CLOCK.dt, 'noise'));
        end
        
    case ('measure') % measurement matrix (groupped H)
        
        % when sensor has bias and this should be estimated, then plug this
        % into the groupped jacobian matrix
        % it is odd. should be capsulated in the RelCartBias class!!!
        jacobian_bias = [];
        for iSensor = 1 : length(obj.SENSOR)
            if strcmp(obj.SENSOR{iSensor}.spec,'RelCartBias')
                jacobian_bias = blkdiag(jacobian_bias, obj.SENSOR{iSensor}.TakeBiasJacobian('BiasMeasure', Y_k{iSensor}));
            else
                % TO DO :: should include additional zeros matrix that length is formulated!! 
            end
        end
        
        jacobian_target = [];
        for iSensor = 1 : length(obj.SENSOR) 
            jacobian_target = [jacobian_target;obj.SENSOR{iSensor}.TakeTargetJacobian(Y_k{iSensor})];
        end
        jacobian = [jacobian_bias, jacobian_target];
        
end

