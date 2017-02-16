function obj = SetParameters( obj, SensedObject, bTrackingTarget, bias, ProcCovMatrix, MeasCovMatrix )
%SETPARAMETERS set parameters in measurement class
%   meausrement covariance matirx is defined by users, and it could also
%   be adjustable before/during simulation

if nargin > 1 % overload process (should be modified in a better way)
	% set process noise covariance for bias
    obj.Q = ProcCovMatrix; % assume constant process noise for bias
    
    % sensor bias
    obj.bias = bias;
    
    % set measurement noise covariance
    obj.R = MeasCovMatrix;
    
    % sensed object setting
    obj.subject = SensedObject;
    
    % set binary array for tracking target index
    obj.bTrack = bTrackingTarget;
end

end

