function [ptNext,wNext] = ResampleParticle(ptNow,wNow,field)

nPt = length(ptNow(1,:));
nState = length(ptNow(:,1));
ptNext = nan(nState,nPt);

for iPt = 1:nPt
    bSample = 1;
    while (bSample)
        % trick: resampled particles should has its own unique location
        % so "small" random value was added to particle's state
        ptNext(:,iPt) = ptNow(:,find(rand <= cumsum(wNow),1)) + mvnrnd(zeros(nState,1),(1e-5)*eye(nState))';
        if IsParticleInBoundary(ptNext(:,iPt),field.boundary) % if the new particle is not on the field
            bSample = 0;
        end
    end
end

% update weight: uniform because particles lose info
wNext = (1/nPt)*ones(1,nPt);

end

% discrimation of the situation whether the reampled particle is on the
% field
function bLoc = IsParticleInBoundary(pt,fieldBound)
    bLocX = pt(1) >= fieldBound(1) && pt(1) <= fieldBound(2);
    bLocY = pt(2) >= fieldBound(3) && pt(2) <= fieldBound(4);
    
    bLoc = bLocX && bLocY;
end