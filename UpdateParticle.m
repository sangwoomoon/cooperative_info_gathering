function ptNew = UpdateParticle(ptNow,param,dt)

nPt = length(ptNow(1,:));
nState = length(ptNow(:,1));
ptNew = nan(nState,nPt);

for iPt = 1:nPt
    bParticleOutOfField = 1;
    while (bParticleOutOfField)
        ptNew(:,iPt) = UpdateTargetState(ptNow(:,iPt),param,dt);
        if CheckParticleInBoundary(ptNew(:,iPt),param.field.boundary)
            bParticleOutOfField = 0;
        end
    end
end

end

function bParticleInBound = CheckParticleInBoundary(pt,boundary)

bParticleInBound = (pt(1)>boundary(1) && pt(1)<boundary(2)) && (pt(2)>boundary(3) && pt(2)<boundary(4));

end