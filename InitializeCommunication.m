function comm = InitializeCommunication(iAgent,sim)

nAgent = sim.nAgent;

comm.id = iAgent;
comm.beta = nan(nAgent,1);
comm.bConnect = nan(nAgent,1);
comm.hist.beta(:,1) = comm.beta; % for all agent and agent itself
comm.hist.bConnect(:,1) = comm.bConnect; % for all agent and agent itself

end