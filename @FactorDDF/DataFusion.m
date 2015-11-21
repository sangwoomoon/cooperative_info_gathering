function o = DataFusion(o, AGENT, SIMULATION, CLOCK, option)

tl = AGENT.FDDF_KF.nState-sum(AGENT.bKFs);

o.XhatTemp = AGENT.FDDF_KF.Xhat;
o.PhatTemp = AGENT.FDDF_KF.Phat;

Mtemp = zeros(tl);
mtemp = zeros(tl,1);

omegaCandi = 0:0.01:1;

for iMerge = 1:SIMULATION.nAgent
    
    if (AGENT.COMM.C(iMerge,AGENT.id) == 1) % communication is connected 
        
        % step 1 :: marginalize the matrix (extract target state)
        o.XhatMgn = o.XhatTemp(1:tl);
        o.PhatMgn = o.PhatTemp(1:tl,1:tl);
        
        % step 2 :: make the optimal solution for omega
        
        if strcmpi(option,'MMNB')
            %%Compute NB pdf for marginal fusion only
            M_mmse = inv(inv(o.PhatMgn) ...
                + inv(AGENT.COMM.Z(iMerge).Phat(1:tl,1:tl)));
            m_mmse = M_mmse*(inv(o.PhatMgn)*o.XhatMgn...
                + inv(AGENT.COMM.Z(iMerge).Phat(1:tl,1:tl))*AGENT.COMM.Z(iMerge).Xhat(1:tl));
        end
        
        for iOmega = 1 : length(omegaCandi);
            
            MCandi(:,:,iOmega) = omegaCandi(iOmega)*inv(o.PhatMgn)...
            + (1-omegaCandi(iOmega))*inv(AGENT.COMM.Z(iMerge).Phat(1:tl,1:tl));
            mCandi(:,iOmega) = inv(MCandi(:,:,iOmega))*...
                ( omegaCandi(iOmega)*inv(o.PhatMgn)*o.XhatMgn...
            + (1-omegaCandi(iOmega))*inv(AGENT.COMM.Z(iMerge).Phat(1:tl,1:tl))*AGENT.COMM.Z(iMerge).Xhat(1:tl) );
            
            switch(option)
                case {'trace'}
                    
                    % weird thing happened : it seems to generate right
                    % result when nAgent = 2, but it fails to estimate if
                    % nAgent >=3.
                    % The only difference between this method and MMSE is
                    % to find the cost function. It is natural and
                    % an disadvantage?
                    Cost(iOmega) = trace(inv(MCandi(:,:,iOmega)));
                    
                case {'MMNB'}
                    
                    Cost(iOmega) = 0.5*( trace(MCandi(:,:,iOmega)*M_mmse) ...
                         + (mCandi(:,iOmega)-m_mmse)'*MCandi(:,:,iOmega)*(mCandi(:,iOmega)-m_mmse)...
                         - length(m_mmse) ...
                         + log( det(inv(MCandi(:,:,iOmega))) / det(M_mmse) ) );
            end
        end
        
        [MinCost,MinIdx] = min(Cost);
        
%         MinIdx = 11; % 0.1
        
        MtempDelta = MCandi(:,:,MinIdx)-inv(o.PhatMgn);
        mtempDelta = MCandi(:,:,iOmega)*mCandi(:,MinIdx)-inv(o.PhatMgn)*o.XhatMgn;
        
        o.omega(iMerge,1) = omegaCandi(MinIdx);
        o.omega(iMerge,2) = 1-omegaCandi(MinIdx);
        
        % step 3 :: Make information matrix / vector from the data fusion with
        % omega (fusion parameter)
        % sequential process (like Batch Process)
        Mtemp = Mtemp + MtempDelta;
        mtemp = mtemp + mtempDelta;
        
        o.M = [Mtemp, zeros(tl,AGENT.FDDF_KF.nState-tl); zeros(AGENT.FDDF_KF.nState-tl,AGENT.FDDF_KF.nState)];
        o.m = [mtemp; zeros(AGENT.FDDF_KF.nState-tl,1)];
        
        o.PhatTemp = inv(inv(AGENT.FDDF_KF.Phat)+o.M);
        o.XhatTemp = o.PhatTemp*(inv(AGENT.FDDF_KF.Phat)*AGENT.FDDF_KF.Xhat+o.m);
        
    else
        
        o.omega(iMerge,1) = nan;
        o.omega(iMerge,2) = nan;
        
    end
end
% 
% o.M = [Mtemp, zeros(tl,AGENT.FDDF_KF.nState-tl); zeros(AGENT.FDDF_KF.nState-tl,AGENT.FDDF_KF.nState)];
% o.m = [mtemp; zeros(AGENT.FDDF_KF.nState-tl,1)];
% 
% o.PhatTemp = inv(inv(AGENT.FDDF_KF.Phat)+o.M);
% o.XhatTemp = o.PhatTemp*(inv(AGENT.FDDF_KF.Phat)*AGENT.FDDF_KF.Xhat+o.m);

o.PhatDDF = o.PhatTemp;
o.XhatDDF = o.XhatTemp;

% store data
o.hist.XhatMgn(:,end+1) = o.XhatMgn;
o.hist.PhatMgn(:,:,end+1) = o.PhatMgn;

o.hist.XhatDDF(:,end+1) = o.XhatDDF;
o.hist.PhatDDF(:,:,end+1) = o.PhatDDF;

o.hist.omega(:,:,end+1) = o.omega;

o.hist.stamp(end+1) = CLOCK.ct;

end