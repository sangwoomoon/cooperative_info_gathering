function obj = SetParameters( obj, SensedObject, bias, MeasCovMatrix )
%SETPARAMETERS set parameters in measurement class
%   meausrement covariance matirx is defined by users, and it could also
%   be adjustable before/during simulation

if nargin > 1 % overload process (should be modified in a better way)
	% set process noise covariance for bias
    obj.Q = zeros(length(bias),1); % assume constant process noise for bias
    
    % sensor bias
    obj.bias = bias;
    
    % set measurement noise covariance
    obj.R = MeasCovMatrix;
    
    % sensed object setting
    obj.subject = SensedObject;
end

end

