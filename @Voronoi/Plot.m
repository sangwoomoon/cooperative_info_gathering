function o = Plot( o )
%PLOT Summary of this function goes here
%   Detailed explanation goes here

for iter = 1 : length(o.cell)
   patch(o.vertices(o.cell{iter},1),o.vertices(o.cell{iter},2),rand(1,3)); hold on;
   alpha(0.1); % set transparency
end

end

