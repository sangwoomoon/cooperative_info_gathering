function obj = Initialize( obj, id )
%INITIALIZEMEASURE allocates initial conditions of measurement for
%measurement data and measurement history

% initialize meausrement data and history : dependent on the measurement
% spec
obj.InitializeData();

% initialize plotting option
obj.InitializePlot(id);

end

