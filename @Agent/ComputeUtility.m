function o = ComputeUtility( o, AGENT, ENVIRONMENT, option, bComm )
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
                        
                        for iAgent = 1 : length(AGENT)
                            
                            if AGENT(iAgent).id ~= o.id
                                utility = utility...
                                    *(1-...
                                    o.ComputeProbComm(AGENT(iAgent).s,AGENT(DFCid).s,bComm)...
                                    *o.ComputeProbDetect(AGENT(iAgent).s,[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)]));
                            end
                            
                        end
                        
                        utility = utility...
                            *o.ComputeProbComm(o.act(iter,:),AGENT(DFCid).s,bComm)*o.ComputeProbDetect(o.act(iter,:),[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)]);
                        
                        o.utilCnd(iter) = o.utilCnd(iter) + utility;
                    
                    end
                    
                end                 
                
            end
        case ('MDFC')
            
            for iter = 1 : length(o.act(:,1))
                
                o.utilCnd(iter) = 0;
                
                for iPointx = 1 : length(ENVIRONMENT.x(1,:))
                    
                    for iPointy = 1 : length(ENVIRONMENT.x(:,1))
                        
                        utilityPART1 = 1;
                        
                        utilityPART2 = 1;
                        
                        utilityPART3 = 1;
                        
                        for iAgentA = 1 : length(AGENT)
                            
                            if AGENT(iAgentA).id ~= o.id
                                
                                for iAgentB = 1 : length(AGENT)
                                    
                                    if AGENT(iAgentB).id ~= o.id
                                        utilityPART1 = utilityPART1...
                                            *(1-...
                                            o.ComputeProbComm(AGENT(iAgentA).s,AGENT(iAgentB).s,bComm)...
                                            *o.ComputeProbDetect(AGENT(iAgentB).s,[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)]));
                                    end
                                    
                                end
                                
                            end
                            
                        end
                        
                        for iAgent = 1 : length(AGENT)
                            
                            utilityPART2 = utilityPART2...
                                *(1-...
                                o.ComputeProbComm(AGENT(iAgent).s,o.act(iter,:),bComm)...
                                *o.ComputeProbDetect(o.act(iter,:),[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)]));
                            
                        end
                        
                        for iAgent = 1 : length(AGENT)
                           
                            if AGENT(iAgent).id ~= o.id
                                utilityPART3 = utilityPART3...
                                    *(1-...
                                    o.ComputeProbComm(AGENT(iAgent).s,o.act(iter,:),bComm)...
                                    *o.ComputeProbDetect(AGENT(iAgent).s,[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)]));
                            end
                            
                        end
                        
                        o.utilCnd(iter) = o.utilCnd(iter) + utilityPART1*(1-utilityPART2*utilityPART3);
                            
                    end
                    
                end
                
            end
            
    end

end

