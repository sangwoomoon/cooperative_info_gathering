function obj = SetParameters( obj, option, AgentID )
%SETPARAMETERS Summary of this function goes here
%   Detailed explanation goes here

% plotting parameters setting (automatically assigned)
switch option
    case 'central'
        obj.plot.htcolor = 'c';
        obj.plot.hbcolor = 'c';
        obj.plot.phatcolor = 'c';
        obj.plot.htmarker = '+';
        obj.plot.hbmarker = '+';
        obj.plot.phatmarker = '.';
        obj.plot.legend = [{'Central xhat'},{'Central Phat'},{'Central Phat'}];
        
    case 'local'
        clr = rand(1,3);
        
        obj.plot.htcolor = clr;
        obj.plot.hbcolor = clr;
        obj.plot.phatcolor = clr;
        obj.plot.htmarker = 'none';
        obj.plot.hbmarker = 'none';
        obj.plot.phatmarker = '--';
        obj.plot.legend = [{strcat('Agent ',num2str(AgentID),' Local xhat')},...
            {strcat('Agent ',num2str(AgentID),' Local Phat')},...
            {strcat('Agent ',num2str(AgentID),' Local Phat')}];
        
    case 'fDDF'
        clr = rand(1,3);

        obj.plot.htcolor = clr;
        obj.plot.hbcolor = clr;
        obj.plot.phatcolor = clr;
        obj.plot.htmarker = 'o';
        obj.plot.hbmarker = 'o';
        obj.plot.phatmarker = '--';
        obj.plot.legend = [{strcat('Agent ',num2str(AgentID),' fDDF xhat')},...
            {strcat('Agent ',num2str(AgentID),' fDDF Phat')},...
            {strcat('Agent ',num2str(AgentID),' fDDF Phat')}];
end

obj.plot.xlabel = {'time (secs)'};
obj.plot.ylabeltarget = [{'Target Easting error (m)'},{'Target Easting Velocity error (m/s)'},...
    {'Target Northing error (m)'},{'Target Northing Velocity error (m/s)'},...
    {'Target Easting error (%)'},{'Target Easting Velocity error (%)'},...
    {'Target Northing error (%)'},{'Target Northing Velocity error (%)'}];
obj.plot.ylabelbias = [{'Sensor Easting Bias error (m)'},{'Sensor Northing Bias error (m)'},...
    {'Sensor Easting Bias error (%)'},{'Sensor Northing Bias error (%)'}];



end

