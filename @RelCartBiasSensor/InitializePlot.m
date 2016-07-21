function obj = InitializePlot( obj, id )
%INITIALIZEPLOT initalize the plotting options of measurements


% set plotting options
obj.plot.color = rand(1,3);

% set plot labels
obj.plot.legend = strcat('Agent ',num2str(id),' meas');


end

