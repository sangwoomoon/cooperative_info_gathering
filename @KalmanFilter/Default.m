
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
% input : empty Decentralized EKF Class
%
% output : set Decentralized EKF Class


o.Phat = [];
o.Xhat = [];
o.nState = 0;

for ii = 1 : length(TARGET)
    o.nState = o.nState + length(TARGET(ii).x);
    o.Xhat = [o.Xhat; TARGET(ii).x];
    o.Phat = blkdiag(o.Phat, 10*eye(length(TARGET(ii).x)));
end

% store initial values
o.hist.Xhat = o.Xhat;
o.hist.Phat = o.Phat;

o.hist.Y = [];
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