function actIdx = ReturnToHome(agentPos)

% take line of sight with respect to the origin
los = atan2(-agentPos(2),-agentPos(1));

heading = wrapToPi(agentPos(3));

if wrapToPi(los-heading) < 0
    actIdx = 3; % turn left
elseif wrapToPi(los-heading) > 0
    actIdx = 2; % turn right
else
    actIdx = 1; % go straight
end

end