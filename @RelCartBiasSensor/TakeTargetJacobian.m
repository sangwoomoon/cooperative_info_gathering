function jacobian = TakeTargetJacobian( obj, MeasTargetID )
%TAKETARGETJACOBIAN generates jacobian matrix for "SINGLE" target
%   this function is used for both centralized and local estimation


switch (obj.meas(MeasTargetID).id)
    case ('LANDMARK')
        jacobian = [0  0  0  0; % H_landmark
                    0  0  0  0];
    otherwise % normal target
        jacobian = [1  0  0  0; % H_target
                    0  0  1  0];
end

end

