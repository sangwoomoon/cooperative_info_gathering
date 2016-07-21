function jacobian = TakeJacobian( obj )
%COMPUTEJACOBIAN computes the jacobian matrix from measurement equation
%   dependent on the sub-classes of Sensor class

jacobian = [];

% for bias part : it depends on the spec of sensor
if strcmp(obj.spec,'RelCartBias') == 1 % when bias is considered
    BiasJacobian = [];
    
    for iMeasure = 1 : length(obj.meas) % assume all measurements are searched (should be modified)
        BiasJacobian = [BiasJacobian; obj.TakeBiasJacobian('BiasMeasure')];
    end
end

StateJacobian = [];

for iMeasure = 1 : length(obj.meas)
    if length(obj.meas(iMeasure).id) == 1 % targets only
        StateJacobian = blkdiag(StateJacobian,obj.TakeTargetJacobian(iMeasure));
    else % landmark (skip if there are NO landmarks)
        m = length(StateJacobian(1,:))/length(obj.TakeTargetJacobian(iMeasure)); % compute scaled factor to fit previous H matrix
        
        LmkJacobian = [];
        for ii = 1 : m
           LmkJacobian = [LmkJacobian,obj.TakeTargetJacobian(iMeasure)];  
        end
        
        StateJacobian = [StateJacobian;LmkJacobian];
    end
end

jacobian = [BiasJacobian, StateJacobian]; % BiasJacobian is not affected on jacobian when it is null

        
end



        

