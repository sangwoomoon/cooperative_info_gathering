function jacobian = GatherJacobian( obj, SENSOR, CLOCK, option )
%GATHERJACOBIANMATRIX GroupS Jacobian matrix with respect to targets and bias

% used input : ESTIMATOR.TARGET.DYNAMICS / ESTIMATOR.SENSOR (if bias exists)

jacobian = [];

switch (option)
    
    case ('state') % state transition (grouped Phi)
        
        % when sensor has bias and this should be estimated, then plug this
        % into the groupped jacobian matrix
        if strcmp(SENSOR.spec,'RelCartBias')
            jacobian = blkdiag(jacobian, SENSOR.TakeBiasJacobian('BiasState'));
        end
        
        for iTarget = 1 : length(obj.TARGET)
            jacobian = blkdiag(jacobian, obj.TARGET(iTarget).DYNAMICS.TakeJacobian(obj.TARGET(iTarget).CONTROL.u, CLOCK.dt, 'state'));
        end

    case ('noise') % process noise (grouped Gamma)
        
        % when sensor has bias and this should be estimated, then plug this
        % into the groupped jacobian matrix
        if strcmp(SENSOR.spec,'RelCartBias')
            jacobian = blkdiag(jacobian, SENSOR.TakeBiasJacobian('BiasNoise'));
        end
        
        for iTarget = 1 : length(obj.TARGET)
            jacobian = blkdiag(jacobian, obj.TARGET(iTarget).DYNAMICS.TakeJacobian(obj.TARGET(iTarget).CONTROL.u, CLOCK.dt, 'noise'));
        end

end

