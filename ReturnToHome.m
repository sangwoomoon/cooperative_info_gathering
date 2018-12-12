function actIdx = ReturnToHome(agentPos)

% take line of sight with respect to the origin
los = atan2(-agentPos(2),-agentPos(1));

if los < 0
    actIdx = 2; % turn right
elseif los > 0
    actIdx = 3; % turn left
else
    actIdx = 1; % go straight
end

end