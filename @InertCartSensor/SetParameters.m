function obj = SetParameters( obj, SensedObject, bTrackingTarget, MeasCovMatrix )
%SETPARAMETERS set parameters in measurement class
%   meausrement covariance matirx is defined by users, and it could also
%   be adjustable before/during simulation

if nargin > 1 % overload process (should be modified in a better way)
    
    % set binary array for tracking target index
    obj.bTrack = bTrackingTarget;
    
    % set measurement noise covariance
    obj.R = MeasCovMatrix;
    
    % sensed object setting
    obj.subject = SensedObject;
    
end

end

