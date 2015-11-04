
% KF for both platform states and target position:

% 1. CENTRALIZED FORM
% % State, measurement and input vector definitions:
% % xhat = [xt_1; xt_2;...;xt_m; xp_1;xp_2;...;xp_n]
% %          = [et_1;nt_1;et_2;nt_2;...et_m;nt_m; ep_1;ep_1dot;np_1;np_1dot; ... ep_n; ep_ndot; np_n; np_ndot];
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

function o = Default ( o , SIMULATION, AGENT, TARGET, CLOCK, option )

% default setting for targets
% input : empty CentralKF Class
%
% output : set CentralKF Class

tl=length(TARGET(1).x);  % number of state for targets
al=length(AGENT(1).s);   % number of state for agents
yl=length(AGENT(1).MEASURE.y); % number of measurements

o.nState = al*length(AGENT)+tl*length(TARGET);           % one target to all agents
o.nY = yl*length(AGENT);                    % NEED TO FIND OUT!

o.F = [];
o.Gamma = [];
o.Gu = [];
o.H = [];  

HpartTgt = [eye(tl*SIMULATION.nTarget);zeros(tl,tl*SIMULATION.nTarget)];
HpartTgtk = [];
HpartAgt = [];
HpartAgtk = [];

o.Q = [];
o.R = [];
o.u = [];

o.Xhat = [];
o.Phat = [];

o.hist.Y = [];
o.hist.Xhat = [];
o.hist.Phat = [];

for iTgtAgt = 1 : length(TARGET)+length(AGENT) % w.r.t total target - total agent (centralized case)
    if iTgtAgt < length(TARGET)+1 % target index
        o.Xhat = [o.Xhat; TARGET(iTgtAgt).x];
        
        o.F = blkdiag(o.F,TARGET(iTgtAgt).Ft);
        o.Gamma = blkdiag(o.Gamma,TARGET(iTgtAgt).Gt); 
        o.Q = blkdiag(o.Q,TARGET(iTgtAgt).Qt); 
    else % agent index
        o.Xhat = [o.Xhat; AGENT(iTgtAgt-length(TARGET)).s];
        
        o.F = blkdiag(o.F,AGENT(iTgtAgt-length(TARGET)).Fp);             % [Ft_{iTarget} 0 0; 0 Fp_1 0; 0 0 Fp_2]
        o.Gamma = blkdiag(o.Gamma,AGENT(iTgtAgt-length(TARGET)).Gamp);   % [Gt_{iTarget} 0 0; 0 Gamp_1 0; 0 0 Gamp_2]
        o.Gu = blkdiag(o.Gu,AGENT(iTgtAgt-length(TARGET)).Gu);             % [Gu1, zeros(4,2); zeros(4,2),Gu2]
        o.Q = blkdiag(o.Q,AGENT(iTgtAgt-length(TARGET)).Qp);
    end
end
o.Gu = [zeros(tl*SIMULATION.nTarget,length(o.Gu(1,:)));o.Gu]; % no input for target

for iAgent = 1 : length(AGENT)
   
   for iTarget = 1 : SIMULATION.nTarget
       o.R = blkdiag(o.R,AGENT(iAgent).MEASURE.Rt{iTarget});
       HpartAgt = [HpartAgt;-AGENT(iAgent).MEASURE.Hp];
   end
   
   o.R = blkdiag(o.R,AGENT(iAgent).MEASURE.Rp);
   
   HpartAgt = [HpartAgt;AGENT(iAgent).MEASURE.Hp];
   HpartAgtk = blkdiag(HpartAgtk,HpartAgt);
   HpartAgt = [];
   
   HpartTgtk = [HpartTgtk; HpartTgt];
end

o.H = [HpartTgtk, HpartAgtk];
o.Phat = 100*eye(o.nState);                                           

switch option
    case 'central'
        o.plot.htcolor = 'b';
        o.plot.hpcolor = 'b';
        o.plot.phatcolor = 'm';
        o.plot.htmarker = 'square';
        o.plot.hpmarker = 'square';
        o.plot.phatmarker = '.';
        o.plot.legend = [{'Central KF xhat'},{'Central KF Phat'},{'Central KF Phat'}];
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
    case 'decentral'
        o.plot.htcolor = rand(1,3);
        o.plot.hpcolor = rand(1,3);
        o.plot.phatcolor = rand(1,3);
        o.plot.htmarker = '+';
        o.plot.hpmarker = '+';
        o.plot.phatmarker = '--';
        o.plot.legend = [{strcat('Agent ',num2str(AGENT(SIMULATION.iAgent).id),' Decentral KF xhat')},...
            {strcat('Agent ',num2str(AGENT(SIMULATION.iAgent).id),' Decentral KF Phat')},...
            {strcat('Agent ',num2str(AGENT(SIMULATION.iAgent).id),' Decentral KF Phat')}];
end

end