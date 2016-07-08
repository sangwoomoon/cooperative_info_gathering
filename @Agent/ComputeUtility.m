function o = ComputeUtility( o, AGENT, ENVIRONMENT, option )
%COMPUTEUTILITY Summary of this function goes here
%   Detailed explanation goes here

    switch (option)
        case ('SDFC')
            
            for iAgent = 1 : length(AGENT)
                if AGENT(iAgent).bDFC == 1;
                    DFCid = AGENT(iAgent).id;
                end
            end

            for iter = 1 : length(o.act(:,1))
                
                o.utilCnd(iter) = 0;
                
                for iPointx = 1 : length(ENVIRONMENT.x(1,:))
                    
                    for iPointy = 1 : length(ENVIRONMENT.x(:,1))
                        
                        utility = 1;
                        
                        for iAgent = 1 : length(AGENT)-1
                            
                            if AGENT(iAgent).id ~= o.id
                                utility = utility...
                                    *(1-...
                                    o.ComputeProbComm(AGENT(iAgent).s,AGENT(DFCid).s)...
                                    *o.ComputeProbDetect(AGENT(iAgent).s,[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)]));
                            end
                            
                        end
                        
                        utility = utility...
                            *o.ComputeProbComm(o.act(iter,:),AGENT(DFCid).s)*o.ComputeProbDetect(o.act(iter,:),[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)]);
                        
                        o.utilCnd(iter) = o.utilCnd(iter) + utility;
                    
                    end
                    
                end                 
                
            end
        case ('mmc')
            
    end

end

