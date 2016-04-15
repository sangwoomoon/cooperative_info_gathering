function o = Default( o, CLOCK )
%DEFAULT Summary of this function goes here
%   Detailed explanation goes here

o.hist.C = [];

o.plot.statecolor = rand(1,3);
o.plot.marker = ['s';'d']; % start; end
o.plot.markersize = 3;
o.plot.linewidth = 3;

% o.plot.legend = [{strcat('Centroid ',num2str(o.id))},...
%     {strcat('Centroid ',num2str(o.id),' start')},...
%     {strcat('Centroid ',num2str(o.id),' end')}];

end

