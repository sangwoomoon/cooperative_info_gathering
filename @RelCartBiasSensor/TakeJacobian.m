function jacobian = TakeJacobian( obj )
%COMPUTEJACOBIAN computes the jacobian matrix from measurement equation
%   dependent on the sub-classes of Sensor class

jacobian = [obj.TakeBiasJacobian('BiasMeasure'), obj.TakeTargetJacobian()];

    
end





