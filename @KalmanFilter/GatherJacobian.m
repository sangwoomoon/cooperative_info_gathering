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
                
        ptRow = 0; % index pointer with respect to row
        ptTgtCol = 0 ; % index pointer with respect to col (target part)
        ptBiasCol = 0; % index pointer with respect to col (bias part)

        
        for iSensor = 1 : length(obj.SENSOR)
            
            % make starting indices with respect to all targets
            for iTarget = 2 : length(obj.TARGET)
                ptTgtCol(iTarget) = ptTgtCol(iTarget-1) + obj.TARGET(iTarget).DYNAMICS.nState;
            end
            
            if length(obj.SENSOR) == 1 % local
                for iTrack = 1 : length(obj.SENSOR{1}.bTrack)
                    if obj.SENSOR{1}.bTrack(iTrack) == 0 % match the indices of targets in local framework
                        ptTgtCol(iTrack+1:end+1) = ptTgtCol(iTrack:end);
                    end
                end
            end
            
            if strcmp(obj.SENSOR{iSensor}.spec,'RelCartBias') % for bias sensor
                                
                % initialize starting index for targets
                nTotalBiasState = 0;
                for iSensorTemp = 1 : length(obj.SENSOR)
                    nTotalBiasState = nTotalBiasState + obj.SENSOR{iSensor}.nState;
                end
                ptTgtCol = ptTgtCol + nTotalBiasState; % indices are shifted by all biases
                
                % with respect to targets
                for iTarget = 1 : sum(obj.SENSOR{iSensor}.bTrack) % for tracked targets only (same as centralized case)
                    
                    dRow = length(Y_k{iSensor}(iTarget).y); % measurement state for single target from single agent
                    
                    % bias part
                    dBiasCol = obj.SENSOR{iSensor}.nState;
                    
                    jacobian( ptRow+1 : ptRow+dRow , ptBiasCol+1 : ptBiasCol+dBiasCol ) = ...
                        obj.SENSOR{iSensor}.TakeBiasJacobian('BiasMeasure');
                    
                    % target part
                    dTgtCol = obj.TARGET(iTarget).DYNAMICS.nState;
                    id = Y_k{iSensor}(iTarget).id;
                    
                    jacobian( ptRow+1 : ptRow+dRow , ptTgtCol(id)+1 : ptTgtCol(id)+dTgtCol ) = ...
                        obj.SENSOR{iSensor}.TakeTargetJacobian();
                    
                    ptRow = ptRow + dRow;
                    
                end
                
                % with respect to landmarks
                for iLandmark = 1 : length(Y_k{iSensor}) - sum(obj.SENSOR{iSensor}.bTrack) % since # of landmarks = {# of measurements} - {# of tracked targets}
                    dRow = length(Y_k{iSensor}(iTarget+iLandmark).y); % measurement state for single target from single agent
                    
                    % bias part
                    dBiasCol = obj.SENSOR{iSensor}.nState;
                    jacobian( ptRow+1 : ptRow+dRow , ptBiasCol+1 : ptBiasCol+dBiasCol ) = ...
                        obj.SENSOR{iSensor}.TakeBiasJacobian('BiasMeasure');
                    
                    ptRow = ptRow + dRow;
                    
                end
               
            else
                
                % TO DO :: other sensor types!
                
            end
            
            ptBiasCol = ptBiasCol + dBiasCol; 
            ptTgtCol = ptTgtCol - nTotalBiasState; % indices are reset
            
        end
        
end

        
end

