function obj = TakeProcess(obj, xhat, Phat, Z, bTrack, current_time, option)


nTotalState = length(xhat);
nTargetState = length(obj.hist.xhatMgn(:,1,1));
nBiasState = nTotalState-nTargetState;

obj.xhatTemp = xhat;
obj.PhatTemp = Phat;

omegaCandidate = 0:0.01:1;

for iMerge = 1 : length(obj.hist.omega(:,1)) % number of agents
    
    if ~isempty(Z{iMerge}) % communication is connected 
        
        % step 0 :: split common targets and ownship targets
        [commonStateIdx_i,commonStateIdx_j, exclusiveStateIdx] = obj.TakeCommonExclusiveStateIndex(bTrack,Z{iMerge}.bTrack);
          
        [obj.xhatMgn, obj.PhatMgn] = obj.TakeMarginalEstimation(commonStateIdx_i, obj.xhatTemp, obj.PhatTemp);
        [xhatMgn_j, PhatMgn_j] = obj.TakeMarginalEstimation(commonStateIdx_j, Z{iMerge}.xhat, Z{iMerge}.Phat);
        
        [obj.xhatTemp, obj.PhatTemp] = obj.PermuteEstimation(commonStateIdx_i,exclusiveStateIdx, obj.xhatTemp, obj.PhatTemp);
        
        % step 2 :: make the optimal solution for omega
        
        if strcmpi(option,'MMNB')
            %%Compute NB pdf for marginal fusion only
            M_mmnb = inv(inv(obj.PhatMgn) + inv(PhatMgn_j));
            m_mmnb = M_mmnb*(inv(obj.PhatMgn)*obj.xhatMgn + inv(PhatMgn_j)*xhatMgn_j);
        end
        
        % reset candidate estimation and covariance matrix (for changable
        % number of common/ownship bias/targets)
        MCandi = [];
        mCandi = [];
        Cost = [];
        
        for iOmega = 1 : length(omegaCandidate)
            
            MCandi(:,:,iOmega) = omegaCandidate(iOmega)*inv(obj.PhatMgn) + (1-omegaCandidate(iOmega))*inv(PhatMgn_j);
            mCandi(:,iOmega) = inv(MCandi(:,:,iOmega))*( omegaCandidate(iOmega)*inv(obj.PhatMgn)*obj.xhatMgn...
            + (1-omegaCandidate(iOmega))*inv(PhatMgn_j)*xhatMgn_j );
            
            switch(option)
                case {'trace'}
                    
                    Cost(iOmega) = trace(inv(MCandi(:,:,iOmega)));
                    
                case {'MMNB'}
                    
                    % with Kullback-Leibler divergence (in multivariate
                    % normal distribution)
                    Cost(iOmega) = 0.5*( trace(MCandi(:,:,iOmega)*M_mmnb) ...
                         + (mCandi(:,iOmega)-m_mmnb)'*MCandi(:,:,iOmega)*(mCandi(:,iOmega)-m_mmnb)...
                         - length(m_mmnb) ...
                         + log( det(inv(MCandi(:,:,iOmega))) / det(M_mmnb) ) );
            
            end
        end
        
        [~,minIdx] = min(Cost);
                
        MtempDelta = MCandi(:,:,minIdx)-inv(obj.PhatMgn);
        mtempDelta = MCandi(:,:,minIdx)*mCandi(:,minIdx)-inv(obj.PhatMgn)*obj.xhatMgn;
  
        obj.omega(iMerge,1) = omegaCandidate(minIdx);
        obj.omega(iMerge,2) = 1-omegaCandidate(minIdx);
        
        % step 3 :: Make information matrix / vector from the data fusion with
        % omega (fusion parameter)
        % sequential process (like Batch Process)        
        nOwnshipState = nBiasState + nTargetState/sum(bTrack)*(length(exclusiveStateIdx(:,1))-1); % assume that all estimated targets are homogeneous (should be modified!!)
        nCommonState = nTotalState - nOwnshipState;
        
        obj.M = [zeros(nOwnshipState,nTotalState);zeros(nCommonState,nOwnshipState),MtempDelta];
        obj.m = [zeros(nOwnshipState,1);mtempDelta];
        
        PhatStore = obj.PhatTemp; % temporary store for computation below (for obj.xhatTemp)
        obj.PhatTemp = inv(inv(PhatStore)+obj.M); 
        obj.xhatTemp = obj.PhatTemp*(inv(PhatStore)*obj.xhatTemp+obj.m);
        
        [obj.xhatTemp,obj.PhatTemp] = obj.RetrievePermutedEstimation(commonStateIdx_i,exclusiveStateIdx);
        
    else
        
        obj.omega(iMerge,1) = nan;
        obj.omega(iMerge,2) = nan;
        
    end
    
end

obj.Phat = obj.PhatTemp;
obj.xhat = obj.xhatTemp;

% store data
obj.hist.xhat(:,end+1) = obj.xhat;
obj.hist.Phat(:,:,end+1) = obj.Phat;

obj.hist.omega(:,:,end+1) = obj.omega;

obj.hist.stamp(end+1) = current_time;

end