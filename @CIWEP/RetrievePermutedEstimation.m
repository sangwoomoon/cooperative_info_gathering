function [ xhat_new, Phat_new ] = RetrievePermutedEstimation( obj, commonStateIdx, exclusiveStateIdx )
%RETRIEVEPERMUTEDESTIMATION re-organizes permuted estimation data for
%fusion process to get a matrix its order of entry is before permutation

retrievedIdx = [exclusiveStateIdx;commonStateIdx]; % re-arrange indices to make group

% initialize estimation and covariance matrix
Phat_new = nan(length(obj.xhatTemp));
xhat_new = nan(length(obj.xhatTemp),1);


ptRow = 1; % pointer for indexing
for iTarget = 1 : length(retrievedIdx(:,1)) % with respect to row
    
    dRow = retrievedIdx(iTarget,2)-retrievedIdx(iTarget,1);
    ptCol = 1; % pointer for indexing
    
    for jTarget = 1 : length(retrievedIdx(:,1)) % with respect to col
        
        dCol = retrievedIdx(jTarget,2)-retrievedIdx(jTarget,1);
        
        Phat_new(retrievedIdx(iTarget,1):retrievedIdx(iTarget,2),retrievedIdx(jTarget,1):retrievedIdx(jTarget,2))=...
            obj.PhatTemp(ptRow:ptRow+dRow,ptCol:ptCol+dCol);
        
        ptCol = ptCol + dCol+1;
    end
    xhat_new(retrievedIdx(iTarget,1):retrievedIdx(iTarget,2)) = obj.xhatTemp(ptRow:ptRow+dRow);
    
    ptRow = ptRow + dRow+1;
end

    


end

