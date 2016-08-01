function obj = TakeProcess(obj, xhat, Phat, Z, current_time, option)

nTotalState = length(xhat);
nTargetState = length(obj.hist.xhatMgn(:,1,1));
nBiasState = nTotalState-nTargetState;

obj.xhatTemp = xhat;
obj.PhatTemp = Phat;

Mtemp = zeros(nTargetState);
mtemp = zeros(nTargetState,1);

omegaCandidate = 0:0.01:1;

for iMerge = 1 : length(obj.hist.omega(:,1)) % number of agents
    
    if ~isempty(Z{iMerge}) % communication is connected 
        
        % step 1 :: marginalize the matrix (extract target state)
        obj.xhatMgn = obj.xhatTemp(nBiasState+1:end);
        obj.PhatMgn = obj.PhatTemp(nBiasState+1:end,nBiasState+1:end);
        
        % step 2 :: make the optimal solution for omega
        
        if strcmpi(option,'MMNB')
            %%Compute NB pdf for marginal fusion only
            M_mmnb = inv(inv(obj.PhatMgn) ...
                + inv(Z{iMerge}.Phat(nBiasState+1:end,nBiasState+1:end)));
            m_mmnb = M_mmnb*(inv(obj.PhatMgn)*obj.xhatMgn...
                + inv(Z{iMerge}.Phat(nBiasState+1:end,nBiasState+1:end))*Z{iMerge}.xhat(nBiasState+1:end));
        end
        
        for iOmega = 1 : length(omegaCandidate);
            
            MCandi(:,:,iOmega) = omegaCandidate(iOmega)*inv(obj.PhatMgn)...
            + (1-omegaCandidate(iOmega))*inv(Z{iMerge}.Phat(nBiasState+1:end,nBiasState+1:end));
            mCandi(:,iOmega) = inv(MCandi(:,:,iOmega))*...
                ( omegaCandidate(iOmega)*inv(obj.PhatMgn)*obj.xhatMgn...
            + (1-omegaCandidate(iOmega))*inv(Z{iMerge}.Phat(nBiasState+1:end,nBiasState+1:end))*Z{iMerge}.xhat(nBiasState+1:end) );
            
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
        
        Mtemp = Mtemp + MtempDelta;
        mtemp = mtemp + mtempDelta;
        
        % step 3 :: Make information matrix / vector from the data fusion with
        % omega (fusion parameter)
        % sequential process (like Batch Process)        
        obj.M = [zeros(nBiasState,nTotalState);zeros(nTargetState,nBiasState),Mtemp];
        obj.m = [zeros(nBiasState,1);mtemp];
        
        obj.PhatTemp = inv(inv(Phat)+obj.M);
        obj.xhatTemp = obj.PhatTemp*(inv(Phat)*xhat+obj.m);
        
    else
        
        obj.omega(iMerge,1) = nan;
        obj.omega(iMerge,2) = nan;
        
    end
    
end

obj.Phat = obj.PhatTemp;
obj.xhat = obj.xhatTemp;

% store data
obj.hist.xhatMgn(:,end+1) = obj.xhatMgn;
obj.hist.PhatMgn(:,:,end+1) = obj.PhatMgn;

obj.hist.xhat(:,end+1) = obj.xhat;
obj.hist.Phat(:,:,end+1) = obj.Phat;

obj.hist.omega(:,:,end+1) = obj.omega;

obj.hist.stamp(end+1) = current_time;

end