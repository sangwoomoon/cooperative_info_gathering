function obj = SetParameter( obj )
%SETPARAMETER sets parameters in NETWORK class or plotting options

nAgent = length(obj.graph(1,:));

obj.plot.xlabel = {'time (secs)'};
obj.plot.ylabel = [];
obj.plot.legend = [];
for iAgent = 1 : nAgent
    obj.plot.lineWidth(iAgent) = 2;
    obj.plot.color(iAgent,:) = rand(1,3);
    obj.plot.ylabel = [obj.plot.ylabel;{strcat('Agent ',num2str(iAgent),' status')}];
    obj.plot.legend = [obj.plot.legend;{strcat('From Agent ',num2str(iAgent))}];
end


end

