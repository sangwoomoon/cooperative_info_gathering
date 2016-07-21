function obj = InitializeData( obj )
%INITIALIZEDATA initialize the measurement data (y) and its history
%   It is overloaded function because measurement vector is varied with
%   respect to the measurement spec (sub-class of measurement class)

obj.meas.id = nan;
obj.meas.y = nan(2,1); % east and north only (should be considered later)

obj.hist.meas{1}.id = obj.meas.id;
obj.hist.meas{1}.y = obj.meas.y;

end

