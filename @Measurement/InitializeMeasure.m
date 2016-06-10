function obj = InitializeMeasure( obj, subject, agentID, id )
%INITIALIZEMEASURE allocates initial conditions of measurement for
%measurement data and measurement history

% initialize meausrement data and history : dependent on the measurement
% spec
obj.InitializeData();

% set who is the measured (measure target? or agent?)
obj.subject = subject;

% set the number of whom to be measured
obj.id = id;

% set plotting options
obj.plot.reltargetcolor = rand(1,3);
obj.plot.absmeasurecolor = rand(1,3);

switch (obj.subject)
    case ('Target')
        obj.plot.legend = strcat('Target ', num2str(obj.id),' meas by Agent ',num2str(agentID));
    case ('Agent')
        obj.plot.legend = strcat('Agent ', num2str(obj.id),' meas by Agent ',num2str(agentID));
end

end

