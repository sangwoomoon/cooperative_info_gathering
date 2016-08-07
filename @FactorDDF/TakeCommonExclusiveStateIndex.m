function [commonStateIdx_i,commonStateIdx_j,exclusiveStateIdx] = TakeCommonExclusiveStateIndex( obj, bTrack_i, bTrack_j )
%TAKECOMMONEXCLUSIVESTATEINDEX finds common states and exclusive (not common) states 
% in terms of indices, which are used for marginal matrix or permutation of
% estimation data

% split common targets and ownship targets based on binary array
bCommonTarget = and(bTrack_i,bTrack_j);
bOwnshipTarget_i = and(bTrack_i, not(bCommonTarget));
bOwnshipTarget_j = and(bTrack_j, not(bCommonTarget));

% compute number of states for indexing
nTotalState = length(obj.xhat);
nTargetState = length(obj.hist.xhatMgn(:,1,1));
nBiasState = nTotalState-nTargetState;

% initalize index-arrays
commonStateIdx_i = [];
commonStateIdx_j = [];

if nBiasState > 0
    exclusiveStateIdx = [1, nBiasState];
else
    exclusiveStateIdx = [];
end

% find indices for common/ownship targets (w.r.t iAgent)
pt = 0; % pointer just for indexing
for iTarget = 1 : length(bTrack_i)
    if bCommonTarget(iTarget) == 1 % if iTarget-th target is common
        pt = pt + 1; 
        commonStateIdx_i(end+1,1) = nBiasState + nTargetState/sum(bTrack_i)*(pt-1)+1; % index starting
        commonStateIdx_i(end,2) = nBiasState + nTargetState/sum(bTrack_i)*pt; % index ending
    elseif bOwnshipTarget_i(iTarget) == 1 % if iTarget-th target is ownship
        pt = pt + 1; 
        exclusiveStateIdx(end+1,1) = nBiasState + nTargetState/sum(bTrack_i)*(pt-1)+1; % index starting
        exclusiveStateIdx(end,2) = nBiasState + nTargetState/sum(bTrack_i)*pt; % index ending
    end
end

% find indices for common targets (w.r.t jAgent)
pt = 0; % pointer just for indexing
for iTarget = 1 : length(bTrack_j)
    if bCommonTarget(iTarget) == 1 % if iTarget-th target for agent j is common
        pt = pt + 1; 
        commonStateIdx_j(end+1,1) = nTargetState/sum(bTrack_i)*(pt-1)+1; % index starting
        commonStateIdx_j(end,2) = nTargetState/sum(bTrack_i)*pt; % index ending
    elseif bOwnshipTarget_j(iTarget) == 1  % otherwise iTarget-th target for agent j is ownship
        pt = pt + 1; 
    end
end



end
