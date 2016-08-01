function jacobian = TakeJacobian( obj )
%COMPUTEJACOBIAN computes the jacobian matrix from measurement equation
%   dependent on the sub-classes of Sensor class

biasPart = [];
targetPart = [];

for iSensor = 1 : length(obj)
    jacobian = [obj.TakeBiasJacobian('BiasMeasure'),obj.TakeTargetJacobian()];
end

    
end





