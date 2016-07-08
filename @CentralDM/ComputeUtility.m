function o = ComputeUtility( o, AGENT, ENVIRONMENT, option )
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
        
        
                
                
                
                for i1 = 1 : length(o.act(1,:,1))
                for i2 = 1 : length(o.act(1,:,1))
                for i3 = 1 : length(o.act(1,:,1))
                for i4 = 1 : length(o.act(1,:,1))
                    
                utility_tot = 0;
                    
                for iPointx = 1 : length(ENVIRONMENT.x(1,:))
            
                    for iPointy = 1 : length(ENVIRONMENT.x(:,1))
                    
                    utility = (1-o.ComputeProbDetect(o.act(1,i1,:),[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)])*...
                        o.ComputeProbComm(o.act(1,i1,:),o.act(DFCid,i3,:)))*...
                        (1-o.ComputeProbDetect(o.act(2,i2,:),[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)])*...
                        o.ComputeProbComm(o.act(2,i2,:),o.act(DFCid,i3,:)))*...
                        (1-o.ComputeProbDetect(o.act(3,i3,:),[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)])*...
                        o.ComputeProbComm(o.act(3,i3,:),o.act(DFCid,i3,:)))*...
                        (1-o.ComputeProbDetect(o.act(4,i4,:),[ENVIRONMENT.x(iPointx,iPointy);ENVIRONMENT.y(iPointx,iPointy)])*...
                        o.ComputeProbComm(o.act(4,i4,:),o.act(DFCid,i3,:)));
                    
                    utility_tot = utility_tot + (1-utility);
                    
                    end
                    
                end
                
                o.utilCnd(i1,i2,i3,i4) = utility_tot;
                
                end
                end
                end
                end
        
        
    case('MDFC')
        
end


end

