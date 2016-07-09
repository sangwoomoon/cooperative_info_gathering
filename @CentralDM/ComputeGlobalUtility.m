function utility_tot = ComputeGlobalUtility( o, x, AGENT, ENVIRONMENT, option )
%COMPUTEGLOBALUTILITY Summary of this function goes here
%   Detailed explanation goes here

utility_tot = 0;

switch(option)
    
    case('SDFC')
        
        for iAgent = 1 : length(o.s(1,:))
            if AGENT(iAgent).bDFC == 1;
                DFCid = AGENT(iAgent).id;
            end
        end
        
        for iPointx = 1 : length(ENVIRONMENT.x(1,:))
            
            for iPointy = 1 : length(ENVIRONMENT.x(:,1))
                
                utility = 1;
                
                for iAgent = 1 : length(AGENT)
                    
                    utility = utility*(1-o.ComputeProbDetect([x(2*(iAgent-1)+1);x(2*iAgent)],[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)])*...
                        o.ComputeProbComm([x(2*(iAgent-1)+1);x(2*iAgent)],[x(2*(DFCid-1)+1);x(2*DFCid)]));
                end
                
                utility_tot = utility_tot + (1-utility);
                
            end
            
            utility_tot = 1/utility_tot;
            
        end
        
        
    case('MDFC')
        
end

end

