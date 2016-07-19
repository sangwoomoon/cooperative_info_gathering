function obj = InitializeData( obj )
%INITIALIZEDATA initialize the measurement data (y) and its history
%   It is overloaded function because measurement vector is varied with
%   respect to the measurement spec (sub-class of measurement class)

obj.y = nan(2,1);
obj.hist.y = nan(2,1);

end

