function obj = InitializeSensor( obj, bias, id )
%INITIALIZEMEASURE allocates initial conditions of measurement for
%measurement data and measurement history

% set subject (target/agent) into sensor class
obj.bias = bias;

% initialize meausrement data and history : dependent on the measurement
% spec
obj.InitializeData();

% initialize plotting option
obj.InitializePlot(id);

end

