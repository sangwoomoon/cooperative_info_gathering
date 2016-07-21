function obj = InitializeNetwork( obj, nAgent, Range )
%INITIALIZESTATE Initialize network 
%   initialized properties :: probability matrix, network graph, range
%   array

obj.prob = nan(nAgent);
obj.graph = nan(nAgent);
obj.range = Range;

obj.hist.prob(1,:,:) = obj.prob;
obj.hist.graph(1,:,:) = obj.graph;

end

