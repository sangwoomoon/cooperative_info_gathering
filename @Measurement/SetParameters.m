function obj = SetParameters( obj, mode, R )
%SETPARAMETERS set parameters in measurement class
%   meausrement covariance matirx is defined by users, and it could also
%   be adjustable before/during simulation

% set measurement action mode
obj.mode = mode;

if nargin > 1 % overload process (should be modified in a better way)
	% set measurement noise covariance
    obj.R = R;
end

end

