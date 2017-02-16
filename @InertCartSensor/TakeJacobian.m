function jacobian = TakeJacobian(obj)
%COMPUTEJACOBIAN computes the jacobian matrix
%   dependent on the sub-classes of Sensor class

jacobian = [1  0  0  0; % H_target
            0  0  1  0];


end





