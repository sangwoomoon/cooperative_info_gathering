% --------------------------------
% plotting function for sensor fov
% --------------------------------
function [xunit,yunit,h] = GetCircleData(x,y,r,clr,opaqueValue)


th = 0:pi/50:2*pi;

xunit = r * cos(th) + x;
yunit = r * sin(th) + y;

if nargin == 5
    h = patch(xunit, yunit, clr);
    alpha(h,opaqueValue);
else
    h = [];
end
