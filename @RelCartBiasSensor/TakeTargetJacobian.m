function jacobian = TakeTargetJacobian( obj )
%TAKETARGETJACOBIAN generates jacobian matrix for "SINGLE" target
%   this function is used for both centralized and local estimation
    
jacobian = [1  0  0  0; % H for single target
            0  0  1  0];

end

