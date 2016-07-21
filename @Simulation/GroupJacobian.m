function jacobian = GroupJacobian( obj, AGENT, TARGET, CLOCK, option )
%GROUPJACOBIANMATRIX Group Jacobian matrix with respect to targets
%   Detailed explanation goes here

% used input : all TARGET.DYNAMICS / all AGENT.SENSOR (if bias exists)

jacobian = [];

switch (option)
    
    case ('state') % state transition (grouped Phi)
        
        % when sensor has bias and this should be estimated, then plug this
        % into the groupped jacobian matrix
        for iAgent = 1 : length(AGENT)
            if strcmp(AGENT(iAgent).SENSOR.spec,'RelCartBias')
                jacobian = blkdiag(jacobian, AGENT(iAgent).SENSOR.TakeBiasJacobian('BiasState'));
            end
        end
        
        for iTarget = 1 : length(TARGET)
            jacobian = blkdiag(jacobian, TARGET(iTarget).DYNAMICS.TakeJacobian(TARGET(iTarget).CONTROL.u, CLOCK.dt, 'state'));
        end

    case ('ProcNoise') % process noise (grouped Gamma)
        
        % when sensor has bias and this should be estimated, then plug this
        % into the groupped jacobian matrix
        for iAgent = 1 : length(AGENT)
            if strcmp(AGENT(iAgent).SENSOR.spec,'RelCartBias')
                jacobian = blkdiag(jacobian, AGENT(iAgent).SENSOR.TakeBiasJacobian('BiasNoise'));
            end
        end
        
        for iTarget = 1 : length(TARGET)
            jacobian = blkdiag(jacobian, TARGET(iTarget).DYNAMICS.TakeJacobian(TARGET(iTarget).CONTROL.u, CLOCK.dt, 'noise'));
        end
        
    case ('measure')
        
        
    case ('MeasNoise')
        
    

end

