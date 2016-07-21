function jacobian = TakeJacobian( obj, MeasTargetID )
%COMPUTEJACOBIAN computes the jacobian matrix
%   dependent on the sub-classes of Sensor class

switch (obj.meas(MeasTargetID).id)
    case ('LANDMARK')
        jacobian = [0  0  0  0; % H_landmark
                    0  0  0  0];
    otherwise % normal target
        jacobian = [1  0  0  0; % H_target
                    0  0  1  0];
end


end





