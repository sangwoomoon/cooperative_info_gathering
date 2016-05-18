
% KF for both platform states and target position:

% 1. CENTRALIZED FORM
% % State, measurement and input vector definitions:
% % xhat = [xt_1; xt_2;...;xt_m]
% %          = [be_1;bn_2;et_1;et1_dot;nt_1;nt1_dot...be_m;bn_m;et_m;etm_dot;nt_m;ntm_dot];
% % z = [z_1;z_2;...;z_n] = [xt_k_rel_1; xp_abs_1; xt_k_rel_2; xp_abs_2 ... ];
% % u = [up_1;up_2;...;up_n] = [ep1_ddot; np1_ddot; ep2_ddot; np2_ddot; ... ; epn_ddot; npn_ddot ];

% 2. LOCAL FORM
% % State, measurement and input vector definitions:
% % xhat = [xt_k; xp_k]
% %          = [et_k;nt_k; ep_k;ep_k_dot;np_k;np_K_dot];
% % z = [z_k] = [xt_k_rel_k; xp_abs_k];
% % u = [up_k] = [ep_k_ddot; np_k_ddot];

% 3. DECENTRALIZED FORM (W.R.T. COMMUNICATION)
% % State, measurement and input vector definitions:
% % xhat = [xt_1;xt_2;...;xt_j; xp_1;xp_2;...;xp_n]
% %          = [et_k;nt_k; ep_1;ep_1dot;np_1;np_1dot; ... ep_n; ep_ndot; np_n; np_ndot];
% % z = [z_1;z_2;...;z_n] = [xt_k_rel_1; xp_abs_1; xt_k_rel_2; xp_abs_2];
% % u = [up_1;up_2;...;up_n] = [ep1_ddot; np1_ddot; ep2_ddot; np2_ddot; ... ; epn_ddot; npn_ddot ];

% 4. fDDF KF FORM
% same as LOCAL FORM

function o = Default ( o , SIMULATION, AGENT, TARGET, CLOCK, option )

% default setting for targets
% input : empty CentralKF Class
%
% output : set CentralKF Class

tl = 0;
al = 0;

for iTgtAgt = 1 : length(AGENT)+length(TARGET)
    if iTgtAgt < length(TARGET)+1
        tl=tl+sum(TARGET(iTgtAgt).bKFx);  % number of state for targets which will be handled in KF
    else
        al=al+sum(AGENT(iTgtAgt-length(TARGET)).DYNAMICS.bKFs);  % number of state for agents which will be handled in KF
    end
end

o.nState = tl+al;           

KFidx = []; % indices of state for KF process

FTempRow = [];
FTemp = [];
o.F = [];

GammaTemp = [];
o.Gamma = [];

o.Gu = [];

HpTemp = [];
HpTempMem = [];
HtTemp = [];
HtTempMem = [];
Hagent = [];
Htarget = [];
o.H = [];  

o.Q = [];
o.R = [];
o.u = [];

o.Xhat = [];
o.Phat = [];

o.hist.Y = [];
o.hist.Xhat = [];
o.hist.Phat = [];

for iAgtTgt = 1 : length(AGENT) + length(TARGET) % w.r.t total agent and target (centralized case)
    
    if iAgtTgt < length(TARGET) + 1 % target index
        
        if sum(TARGET(iAgtTgt).bKFx) ~= 0 % if there are states considered as the entries of KF process,
           
            % filter F,Gamma matrix to make KF process
            for iTgtState = 1 : length(TARGET(iAgtTgt).x)
                if (TARGET(iAgtTgt).bKFx(iTgtState)==1)
                    o.Xhat = [o.Xhat;TARGET(iAgtTgt).x(iTgtState)+(-3+6*rand)]; % priori with noise
                    KFidx = [KFidx, iTgtState];
                end
            end
            
            for iKFstateRow = 1 : length(KFidx)
                for iKFstateCol = 1 : length(KFidx)
                    FTempRow = [FTempRow, TARGET(iAgtTgt).Ft(KFidx(iKFstateRow),KFidx(iKFstateCol))];
                end
                FTemp = [FTemp;FTempRow];
                GammaTemp = [GammaTemp; TARGET(iAgtTgt).Gt(KFidx(iKFstateRow),:)];
                FTempRow = [];
            end
            
            o.F = blkdiag(o.F,FTemp);
            o.Gamma = blkdiag(o.Gamma,GammaTemp);
            FTemp = [];
            GammaTemp = [];
            
            o.Q = blkdiag(o.Q,TARGET(iAgtTgt).Qt);
            KFidx = [];

        end
        
    else % agent index
        
        if sum(AGENT(iAgtTgt-length(TARGET)).DYNAMICS.bKFs) ~= 0 % if there are states considered as the entries of KF process,
            
            % filter F,Gamma matrix to make KF process
            for iAgtState = 1 : length(AGENT(iAgtTgt-length(TARGET)).DYNAMICS.s)
                if (AGENT(iAgtTgt-length(TARGET)).DYNAMICS.bKFs(iAgtState)==1)
                    o.Xhat = [o.Xhat;AGENT(iAgtTgt-length(TARGET)).DYNAMICS.s(iAgtState)+(-3+6*rand)]; % priori with noise
                    KFidx = [KFidx, iAgtState];
                end
            end
            
            for iKFstateRow = 1 : length(KFidx)
                for iKFstateCol = 1 : length(KFidx)
                    FTempRow = [FTempRow, AGENT(iAgtTgt-length(TARGET)).DYNAMICS.Fp(KFidx(iKFstateRow),KFidx(iKFstateCol))];
                end
                FTemp = [FTemp;FTempRow];
                GammaTemp = [GammaTemp; AGENT(iAgtTgt-length(TARGET)).DYNAMICS.Gamp(KFidx(iKFstateRow),:)];
                FTempRow = [];
            end
            
            o.F = blkdiag(o.F,FTemp);
            o.Gamma = blkdiag(o.Gamma,GammaTemp);
            FTemp = [];
            GammaTemp = [];
            
            o.Q = blkdiag(o.Q,AGENT(iAgtTgt-length(TARGET)).DYNAMICS.Qp);
            KFidx = [];
            
        end
        
    end
 
end

for iAgent = 1 : length(AGENT)
    
    % target part
    for iTarget = 1 : length(TARGET)
        
        % SHOULD BE MODIFIDED!!
        % if sum(TARGET(iTarget).bKFx) ~= 0 % if there are states considered as the entries of KF process,
            
            % filter H matrix to make KF process
            for iTgtState = 1 : length(TARGET(iTarget).x)
                if (TARGET(iTarget).bKFx(iTgtState)==1)
                    KFidx = [KFidx, iTgtState];
                end
            end
            
            for iKFstateCol = 1 : length(KFidx)
                HtTempMem = [HtTempMem,AGENT(iAgent).MEASURE(iTarget).Ht(:,KFidx(iKFstateCol))];
            end
            
            if ~isempty(HtTempMem)
                HtTemp = blkdiag(HtTemp,HtTempMem);
            else
                % this target state is not considered (i.e. landmark), so
                % we make the zero matrix below the HtTemp because this
                % measurment is used for the state of agent.
                HtTemp = [HtTemp;zeros(length(HtTemp(:,1))/(iTarget-1),length(HtTemp(1,:)))];
            end
            KFidx = [];
            HtTempMem = [];
            
            %         tricky part. should be re-defined withr respect to the
            %         measurement conditions
            o.R = blkdiag(o.R,AGENT(iAgent).MEASURE(iTarget).Rt);
            
            
            
            if sum(AGENT(iAgent).DYNAMICS.bKFs) ~= 0 % if there are states considered as the entries of KF process,
                
                % filter H matrix to make KF process
                for iTgtState = 1 : length(AGENT(iAgent).DYNAMICS.s)
                    if (AGENT(iAgent).DYNAMICS.bKFs(iTgtState)==1)
                        KFidx = [KFidx, iTgtState];
                    end
                end
                
                for iKFstateCol = 1 : length(KFidx)
                    HpTempMem = [HpTempMem,AGENT(iAgent).MEASURE(iTarget).Hp(:,KFidx(iKFstateCol))];
                end
                HpTemp = [HpTemp;HpTempMem];
                KFidx = [];
                HpTempMem = [];
                
            end
            
        % end

%         tricky part. should be re-defined withr respect to the
%         measurement conditions
%         o.R = blkdiag(o.R,AGENT(iAgent).MEASURE.Rp); 

        
    end
    
    if isempty(Htarget)
        Htarget = HtTemp;
    else
        Htarget = [Htarget; HtTemp];
    end
    
    Hagent = blkdiag(Hagent,HpTemp);
    HtTemp = [];
    HpTemp = [];
    
end

o.H = [Htarget, Hagent];
o.Phat = 10*eye(o.nState);

% store initial values
o.hist.Xhat = o.Xhat;
o.hist.Phat = o.Phat;
o.hist.stamp = 0;

switch option
    case 'central'
        o.plot.htcolor = 'c';
        o.plot.hpcolor = 'c';
        o.plot.phatcolor = 'm';
        o.plot.htmarker = '+';
        o.plot.hpmarker = '+';
        o.plot.phatmarker = '.';
        o.plot.legend = [{'Central KF xhat'},{'Central KF Phat'},{'Central KF Phat'}];
        
        for iAgent = 1 : length(AGENT)
            for iTarget = 1 : length(TARGET)
                o.hist.Y = [o.hist.Y;nan(length(AGENT(iAgent).MEASURE(iTarget).y),1)];
            end
        end
    case 'local'
        o.plot.htcolor = rand(1,3);
        o.plot.hpcolor = rand(1,3);
        o.plot.phatcolor = rand(1,3);
        o.plot.htmarker = 'none';
        o.plot.hpmarker = 'none';
        o.plot.phatmarker = '--';
        o.plot.legend = [{strcat('Agent ',num2str(AGENT.id),' Local KF xhat')},...
            {strcat('Agent ',num2str(AGENT.id),' Local KF Phat')},...
            {strcat('Agent ',num2str(AGENT.id),' Local KF Phat')}];
        
        for iTarget = 1 : length(TARGET)
            o.hist.Y = [o.hist.Y; nan(length(AGENT(1).MEASURE(iTarget).y),1)];
        end
    case 'decentral'
        o.plot.htcolor = rand(1,3);
        o.plot.hpcolor = rand(1,3);
        o.plot.phatcolor = rand(1,3);
        o.plot.htmarker = 'diamond';
        o.plot.hpmarker = 'diamond';
        o.plot.phatmarker = '--';
        o.plot.legend = [{strcat('Agent ',num2str(AGENT.id),' Decentral KF xhat')},...
            {strcat('Agent ',num2str(AGENT.id),' Decentral KF Phat')},...
            {strcat('Agent ',num2str(AGENT.id),' Decentral KF Phat')}];
        
        for iTarget = 1 : length(TARGET)
            o.hist.Y = [o.hist.Y; nan(sum(AGENT.COMM.C(:,AGENT.id))*length(AGENT(1).MEASURE(iTarget).y),1)];
        end
    case 'fDDF'
        o.plot.htcolor = rand(1,3);
        o.plot.hpcolor = rand(1,3);
        o.plot.phatcolor = rand(1,3);
        o.plot.htmarker = 'o';
        o.plot.hpmarker = 'o';
        o.plot.phatmarker = '--';
        o.plot.legend = [{strcat('Agent ',num2str(AGENT.id),' fDDF KF xhat')},...
            {strcat('Agent ',num2str(AGENT.id),' fDDF KF Phat')},...
            {strcat('Agent ',num2str(AGENT.id),' fDDF KF Phat')}];
        
        for iTarget = 1 : length(TARGET)
            o.hist.Y = [o.hist.Y; nan(length(AGENT(1).MEASURE(iTarget).y),1)];
        end
end

o.plot.xlabel = {'time (secs)'};
o.plot.ylabeltarget = [{'Target Easting error (m)'},{'Target Easting Velocity error (m/s)'},...
    {'Target Northing error (m)'},{'Target Northing Velocity error (m/s)'},...
    {'Target Easting error (%)'},{'Target Easting Velocity error (%)'},...
    {'Target Northing error (%)'},{'Target Northing Velocity error (%)'}];
o.plot.ylabelagent = [{'Sensor Easting Bias error (m)'},{'Sensor Northing Bias error (m)'},...
    {'Sensor Easting Bias error (%)'},{'Sensor Northing Bias error (%)'}];

end