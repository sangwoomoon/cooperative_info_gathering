function obj = ComputeCost( obj, histxhat, histPhat, histGraph, AGENT, TARGET, timeFusion, option, iSim, iAgent, bTrack )
%COMPUTECOST computes performance cost for each simulation run
%   The cost is determined by the Phat at the final time-step and defined
%   as the maximum variance of Phat element in all agents

switch (option)
    case ('central')
        iCost = 1;
    case ('MMNB')
        iCost = 2;
    case ('diag')
        iCost = 3;
end

for iClock = 1 : length(histPhat)
            
    iTrack = 0;
    
    if rem(iClock, timeFusion) == 0 % if this time-step is fusion time step

        for iTarget = 1 : length(bTrack(1,:))
        
            if bTrack(iAgent,iTarget) == 1 % if it is tracked target
                
                iTrack = iTrack + 1;
                
                for jAgent = 1 : length(bTrack(:,1))
                   
                    if bTrack(jAgent,iTarget) == 1 % if other agent also track this target
                       
                        if histGraph(iClock,iAgent,jAgent) == 1 % if those two are connected
                            
                            switch (option)
                                
                                case ('central')
                                    
                                    % TRACE fusion estimation storage : target
                                    obj.cost.trace(iAgent,iSim,iTarget,iCost) = trace(squeeze(histPhat(4*(iTarget-1)+7:4*iTarget+6,4*(iTarget-1)+7:4*iTarget+6,iClock)));
                                    
                                    % TRACE fusion estimation storage : bias
                                    obj.cost.trace(iAgent,iSim,4,iCost) = trace(squeeze(histPhat(2*(iAgent-1)+1:2*iAgent,2*(iAgent-1)+1:2*iAgent,iClock)));
                                    
                                    % Norm of posiiton error fusion estimation storage : target
                                    obj.cost.mse(iAgent,iSim,iTarget,iCost) = ((TARGET(iTarget).DYNAMICS.hist.x(1:2,iClock)-...
                                        [histxhat(4*(iTarget-1)+7,iClock);histxhat(4*(iTarget-1)+9,iClock)])'*(TARGET(iTarget).DYNAMICS.hist.x(1:2,iClock)-...
                                        [histxhat(4*(iTarget-1)+7,iClock);histxhat(4*(iTarget-1)+9,iClock)]))^(1/2);
                                    
                                    % Norm of posiiton error fusion estimation storage : bias
                                    if strcmp(AGENT(iAgent).ESTIMATOR(1).SENSOR{1}.spec,'RelCartBias') == 1 % if the sensor considered in the estimation class is RelCartBias
                                        obj.cost.mse(iAgent,iSim,4,iCost) = ((AGENT(iAgent).SENSOR.bias-...
                                            [histxhat(2*(iAgent-1)+1,iClock);histxhat(2*iAgent,iClock)])'*(AGENT(iAgent).SENSOR.bias-...
                                            [histxhat(2*(iAgent-1)+1,iClock);histxhat(2*iAgent,iClock)]))^(1/2);
                                    end
                                    
                                otherwise
                                    
                                    % TRACE fusion estimation storage : target
                                    obj.cost.trace(iAgent,iSim,iTarget,iCost) = trace(squeeze(histPhat(4*(iTrack-1)+3:4*iTrack+2,4*(iTrack-1)+3:4*iTrack+2,iClock)));
                                    
                                    % TRACE fusion estimation storage : bias
                                    obj.cost.trace(iAgent,iSim,4,iCost) = trace(squeeze(histPhat(1:2,1:2,iClock)));
                                    
                                    % Norm of posiiton error fusion estimation storage : target
                                    obj.cost.mse(iAgent,iSim,iTarget,iCost) = ((TARGET(iTarget).DYNAMICS.hist.x(1:2,iClock)-...
                                        [histxhat(4*(iTrack-1)+3,iClock);histxhat(4*(iTrack-1)+5,iClock)])'*(TARGET(iTarget).DYNAMICS.hist.x(1:2,iClock)-...
                                        [histxhat(4*(iTrack-1)+3,iClock);histxhat(4*(iTrack-1)+5,iClock)]))^(1/2);
                                    
                                    % Norm of posiiton error fusion estimation storage : bias
                                    if strcmp(AGENT(iAgent).ESTIMATOR(1).SENSOR{1}.spec,'RelCartBias') == 1 % if the sensor considered in the estimation class is RelCartBias
                                        obj.cost.mse(iAgent,iSim,4,iCost) = ((AGENT(iAgent).SENSOR.bias-histxhat(1:2,iClock))'*...
                                            (AGENT(iAgent).SENSOR.bias-histxhat(1:2,iClock)))^(1/2);
                                    end
                                    
                            end
                            
                            
                        end
                        
                    end
                    
                end
                
            end
            
        end
                
    end
    
end



% switch (option)
%     case ('central')
%         
%         if obj.cost.trace(iAgent,iSim,iTarget,iCost)
%             
%     case ('MMNB')
%         iCost = 2;
%     case ('diag')
%         iCost = 3;
% end


    
