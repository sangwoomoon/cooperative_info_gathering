function o = ComputeGlobalUtility( o, AGENT, ENVIRONMENT, option )
%COMPUTEGLOBALUTILITY Summary of this function goes here
%   Detailed explanation goes here

utility_tot = 0;

switch(option)
    
    case('SDFC')
        
        for iAgent = 1 : length(AGENT)
            if AGENT(iAgent).bDFC == 1;
                DFCid = AGENT(iAgent).id;
            end
        end
        
        for iPointx = 1 : length(ENVIRONMENT.x(1,:))
            
            for iPointy = 1 : length(ENVIRONMENT.x(:,1))
                
                utility = 1;
                
                for iAgent = 1 : length(AGENT)
                    
                    utility = utility*(1-AGENT(iAgent).ComputeProbDetect(AGENT(iAgent).s,[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)])*...
                        AGENT(iAgent).ComputeProbComm(AGENT(iAgent).s,AGENT(DFCid).s));
                end
                
                utility_tot = utility_tot + (1-utility);
                
            end
            
        end
        
        
    case('MDFC')
        
end

o.hist.util(end+1) = utility_tot;

end

