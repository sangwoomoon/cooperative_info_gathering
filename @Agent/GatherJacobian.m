function jacobian = GatherJacobian( obj, TARGET, CLOCK, option )
%GATHERJACOBIANMATRIX GroupS Jacobian matrix with respect to targets
%   Detailed explanation goes here

% used input : TARGET.DYNAMICS / obj.SENSOR (if bias exists)

jacobian = [];

switch (option)
    
    case ('state') % state transition (grouped Phi)
        
        % when sensor has bias and this should be estimated, then plug this
        % into the groupped jacobian matrix
        if strcmp(obj.SENSOR.spec,'RelCartBias')
            jacobian = blkdiag(jacobian, obj.SENSOR.TakeBiasJacobian('BiasState'));
        end
        
        for iTarget = 1 : length(TARGET)
            jacobian = blkdiag(jacobian, TARGET(iTarget).DYNAMICS.TakeJacobian(TARGET(iTarget).CONTROL.u, CLOCK.dt, 'state'));
        end

    case ('noise') % process noise (grouped Gamma)
        
        % when sensor has bias and this should be estimated, then plug this
        % into the groupped jacobian matrix
        if strcmp(obj.SENSOR.spec,'RelCartBias')
            jacobian = blkdiag(jacobian, obj.SENSOR.TakeBiasJacobian('BiasNoise'));
        end
        
        for iTarget = 1 : length(TARGET)
            jacobian = blkdiag(jacobian, TARGET(iTarget).DYNAMICS.TakeJacobian(TARGET(iTarget).CONTROL.u, CLOCK.dt, 'noise'));
        end

end

