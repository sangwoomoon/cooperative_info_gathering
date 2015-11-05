
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

function o = Default ( o , SIMULATION, AGENT, TARGET, CLOCK, option )

% default setting for targets
% input : empty CentralKF Class
%
% output : set CentralKF Class

tl=length(TARGET(1).x);  % number of state for targets

o.nState = tl*length(TARGET);           

o.F = [];
o.Gamma = [];
o.Gu = [];
o.H = [];  

o.Q = [];
o.R = [];
o.u = [];

o.Xhat = [];
o.Phat = [];

o.hist.Y = [];
o.hist.Xhat = [];
o.hist.Phat = [];

for iTarget = 1 : length(TARGET) % w.r.t total target (centralized case)
    o.Xhat = [o.Xhat; TARGET(iTarget).x];
    
    o.F = blkdiag(o.F,TARGET(iTarget).Ft);
    o.Gamma = blkdiag(o.Gamma,TARGET(iTarget).Gt);
    o.Q = blkdiag(o.Q,TARGET(iTarget).Qt); 
end

for iAgent = 1 : length(AGENT)
    for iTarget = 1 : length(TARGET)
        o.R = blkdiag(o.R,AGENT(iAgent).MEASURE.Rt{iTarget});
    end
   o.H = [o.H;AGENT(iAgent).MEASURE.Ht];
end

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