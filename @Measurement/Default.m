function o = Default (o, TARGET, CLOCK, iAgent)

% switch (TARGET.id)
%     case {1,2}
%         o.Hp =[1 0 -1 0 0 0 ;
%                0 1 0 0 -1 0]; % used for input for measurement of target (rel.position with bias)
%         o.Ht =[1 0 0 0;
%                0 0 1 0]; % used for input for measurement of target (rel.position)
%     case {3}
%         o.Hp =[1 0 -1 0 0 0 ;
%                0 1 0 0 -1 0]; % used for input for measurement of target (rel.position with bias)
%         o.Ht =[1 0 0 0;
%                0 0 1 0]; % used for input for measurement of target (rel.position)
% end

for iTarget = 1 : length(TARGET)
    o.y(2*(iTarget-1)+1:2*iTarget) = nan(2,1);
    o.hist.y(2*(iTarget-1)+1:2*iTarget) = nan(2,1);
end

o.y = o.y';             % transpose to make vector form
o.hist.y = o.hist.y';   % transpose to make vector form

o.plot.reltargetcolor = rand(1,3);
o.plot.absmeasurecolor = rand(1,3);

o.plot.legend = strcat('Agent ', num2str(iAgent),' rel. Target ', num2str(TARGET.id),' meas');

end