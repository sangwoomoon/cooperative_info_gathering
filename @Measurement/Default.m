function o = Default (o, TARGET, CLOCK, iAgent)

o.Hp =[1 0 0 0 ;
       0 0 1 0]; % absolute/GPS-like position measurement model for platform

for iTarget = 1 : length(TARGET)
    o.y(2*(iTarget-1)+1:2*iTarget) = nan(2,1);
    o.hist.y(2*(iTarget-1)+1:2*iTarget) = nan(2,1);
end

o.y(2*iTarget+1:2*iTarget+2) = nan(2,1);
o.hist.y(2*iTarget+1:2*iTarget+2) = nan(2,1);

o.y = o.y';
o.hist.y = o.hist.y';

o.plot.reltargetcolor = rand(1,3);
o.plot.absmeasurecolor = rand(1,3);

o.plot.legend = [{strcat('Agent ', num2str(iAgent),' rel. Target 1 meas')},...
    {strcat('agent ', num2str(iAgent),' rel. target 2 meas')},...
    {strcat('agent ', num2str(iAgent),' abs. meas')}];

end