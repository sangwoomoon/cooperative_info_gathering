function o = Default(o, AGENT, SIMULATION)

o.XhatDDF = AGENT.FDDF_KF.Xhat;
o.PhatDDF = AGENT.FDDF_KF.Phat;

tl = AGENT.FDDF_KF.nState-sum(AGENT.DYNAMICS.bKFs);

o.hist.XhatMgn = nan(tl,1);
o.hist.PhatMgn = nan(tl,tl,1);

o.hist.XhatDDF = nan(AGENT.FDDF_KF.nState,1);
o.hist.PhatDDF = nan(AGENT.FDDF_KF.nState,AGENT.FDDF_KF.nState,1);

o.hist.omega = nan(SIMULATION.nAgent,2,1); 

o.hist.stamp = 0;
    
end