function o = ComputeGlobalUtility( o, AGENT, ENVIRONMENT, option )
%COMPUTEGLOBALUTILITY Summary of this function goes here
%   Detailed explanation goes here



switch(option)
    
    case('SDFC')
        
        utility_tot = 0;
        
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
                        AGENT(iAgent).ComputeProbComm(AGENT(iAgent).s,AGENT(DFCid).s,1));
                end
                
                utility_tot = utility_tot + (1-utility);
                
            end
            
        end
        
        
    case('MDFC')
        
        utility_tot = 0;
        
        for iPointx = 1 : length(ENVIRONMENT.x(1,:))
            
            for iPointy = 1 : length(ENVIRONMENT.x(:,1))
                
                utility = 1;
                
                for iAgentA = 1 : length(AGENT)
                    
                    for iAgentB = 1 : length(AGENT)
                    
                    utility = utility*(1-AGENT(iAgentB).ComputeProbDetect(AGENT(iAgentB).s,[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)])*...
                        AGENT(iAgentB).ComputeProbComm(AGENT(iAgentB).s,AGENT(iAgentA).s,1));
                    
                    end
                    
                end
                
                utility_tot = utility_tot + (1-utility);
                
            end
            
        end
        
        
end

o.hist.util(end+1) = utility_tot;

end

