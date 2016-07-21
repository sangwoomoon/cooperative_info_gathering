function obj = SetParameters( obj, SensedObject, bias, biasProcCovMatrix, biasMeasCovMatrix, MeasCovMatrix )
%SETPARAMETERS set parameters in measurement class
%   meausrement covariance matirx is defined by users, and it could also
%   be adjustable before/during simulation

if nargin > 1 % overload process (should be modified in a better way)
	% set process noise covariance for bias
    obj.Q = biasProcCovMatrix;
    
    % sensor bias
    obj.bias = bias;
    
    % set bias measurement noise covariance
    obj.R_bias = biasMeasCovMatrix;
    
    % set measurement noise covariance
    obj.R = MeasCovMatrix;
    
    % sensed object setting
    obj.subject = SensedObject;
end

end

