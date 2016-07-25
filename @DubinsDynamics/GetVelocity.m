function vel = GetVelocity( obj )
%GETPOSITION returns the velocity (east,north) of agent
%   Detailed explanation goes here

if isempty(obj.w)
    vel = zeros(2,1);
else
    vel = [obj.w(1); obj.w(2)]; % x velocity / y velocity
end
% vel = [u(1)*cos(obj.x(3)) + obj.w(1); u(1)*sin(obj.x(3)) + obj.w(2)]; % x velocity / y velocity

end

