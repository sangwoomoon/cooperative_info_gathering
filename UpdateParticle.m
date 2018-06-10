function ptNew = UpdateParticle(ptNow,param,dt)

nPt = length(ptNow(1,:));
nState = length(ptNow(:,1));
ptNew = nan(nState,nPt);

for iPt = 1:nPt
    ptNew(:,iPt) = UpdateTargetState(ptNow(:,iPt),param,dt);
end

end