function o = DataFusionAll(o, AGENT, SIMULATION, CLOCK)

tl = AGENT.FDDF_KF.nState-sum(AGENT.bKFs);

% step 1 :: marginalize the matrix (extract target state)
o.XhatMgn = AGENT.FDDF_KF.Xhat(1:tl);
o.PhatMgn = AGENT.FDDF_KF.Phat(1:tl,1:tl);

% step 2 :: Make information matrix / vector from the data fusion with
% omega (fusion parameter)

Mtemp = (o.omega(AGENT.id)-1)*inv(o.PhatMgn);
mtemp = (o.omega(AGENT.id)-1)*inv(o.PhatMgn)*o.XhatMgn;

for iMerge = 1:SIMULATION.nAgent
   if AGENT.COMM.C(iMerge,AGENT.id) == 1 % communication is connected
       
      % step 3 :: TO DO LIST (make the optimal solution for omega)

      Mtemp = Mtemp + o.omega(iMerge)*inv(AGENT.COMM.Z(iMerge).Phat(1:tl,1:tl));
      mtemp = mtemp + o.omega(iMerge)*inv(AGENT.COMM.Z(iMerge).Phat(1:tl,1:tl))*AGENT.COMM.Z(iMerge).Xhat(1:tl);
   end
end

o.M = [Mtemp, zeros(tl,AGENT.FDDF_KF.nState-tl); zeros(AGENT.FDDF_KF.nState-tl,AGENT.FDDF_KF.nState)];
o.m = [mtemp; zeros(AGENT.FDDF_KF.nState-tl,1)];

o.PhatDDF = inv(inv(AGENT.FDDF_KF.Phat)+o.M);
o.XhatDDF = o.PhatDDF*(inv(AGENT.FDDF_KF.Phat)*AGENT.FDDF_KF.Xhat+o.m);

% store data
o.hist.XhatMgn(:,end+1) = o.XhatMgn;
o.hist.PhatMgn(:,:,end+1) = o.PhatMgn;
o.hist.XhatDDF(:,end+1) = o.XhatDDF;
o.hist.PhatDDF(:,:,end+1) = o.PhatDDF;
o.hist.stamp(end+1) = CLOCK.ct;

end