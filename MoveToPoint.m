function actIdx = MoveToPoint(PTset, agentPos)

nTarget = length(PTset);
targetsPos = nan(length(PTset.xhat),nTarget);

for iTarget = 1 : nTarget
    targetsPos(:,iTarget) = PTset.xhat;
end

% take mean of targets' positions
targetPos = mean(targetsPos,2);

los = atan2(targetPos(2)-agentPos(2),targetPos(1)-agentPos(1));

heading = wrapToPi(agentPos(3));

if wrapToPi(los-heading) < 0
    actIdx = 2; % turn right
elseif wrapToPi(los-heading) > 0
    actIdx = 3; % turn left
else
    actIdx = 1; % go straight
end

end