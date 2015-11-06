function o = Default (o, TARGET, CLOCK, iAgent)

o.Hp =[1 0 0 0 ;
       0 0 1 0]; % used for input for measurement of target (rel.position)
   
o.Ht =[1 0 0 0;
       0 0 1 0]; % used for input for measurement of target (rel.position) without bias
   
o.Hb = eye(2); % used for input for measurement of target (rel.position) bias
   

for iTarget = 1 : length(TARGET)
    o.y(2*(iTarget-1)+1:2*iTarget) = nan(2,1);
    o.hist.y(2*(iTarget-1)+1:2*iTarget) = nan(2,1);
end

o.y = o.y';             % transpose to make vector form
o.hist.y = o.hist.y';   % transpose to make vector form

o.plot.reltargetcolor = rand(1,3);
o.plot.absmeasurecolor = rand(1,3);

o.plot.legend = [{strcat('Agent ', num2str(iAgent),' rel. Target 1 meas')},...
    {strcat('agent ', num2str(iAgent),' rel. target 2 meas')}];

end