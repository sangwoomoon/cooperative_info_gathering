function obj = SetParameters( obj, biasQinput, SensedObject, R )
%SETPARAMETERS set parameters in measurement class
%   meausrement covariance matirx is defined by users, and it could also
%   be adjustable before/during simulation

if nargin > 1 % overload process (should be modified in a better way)
	% set process noise covariance for bias
    obj.Q = biasQinput;
    
    % set measurement noise covariance
    obj.R = R;
    
    obj.subject = SensedObject;
end

end

