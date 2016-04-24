function o = Plot(o)
%PLOT Summary of this function goes here
%   Detailed explanation goes here

for iAgent = 1 : length(o.cell)
    line(o.vertices(o.cell{iAgent},1),o.vertices(o.cell{iAgent},2)); hold on;
end

end

