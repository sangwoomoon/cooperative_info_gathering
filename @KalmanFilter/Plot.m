function obj = Plot (obj, AGENT, TARGET, CLOCK, SIM, option)


nBiasState=length(AGENT(1).SENSOR.bias); 
nTgtState=(AGENT(1).ESTIMATOR.nState - nBiasState)/SIM.nTarget;

BiasList = [];
TargetList = [];

% make index of subplot : bias state
for iBiasState = 1 : nBiasState
    BiasList = [BiasList, nBiasState*100+10+iBiasState];
end

% make index of subplot : target state
for iTgtState = 1 : nTgtState
    TargetList = [TargetList, nTgtState*100+10+iTgtState];
end


% Target Error Plot
for iTarget = 1 : length(TARGET)
    
    % Target state errors:
    figure(iTarget+1), hold on;
    if strcmp(option,'central') % should be moidified! (to central)
        suptitle(['Target ',num2str(iTarget), ' State Estimation Errors'])
    end
    
    for iKFstate = 1 : nTgtState
        subplot(TargetList(iKFstate)), hold on;
        plot(CLOCK.tvec,obj.hist.xhat(nTgtState*(iTarget-1)+iKFstate+nBiasState,2:end)-TARGET(iTarget).DYNAMICS.hist.x(iKFstate,2:end),'marker',obj.plot.htmarker,'color',obj.plot.htcolor);
        plot(CLOCK.tvec,2*sqrt(squeeze(obj.hist.Phat(nTgtState*(iTarget-1)+iKFstate,nTgtState*(iTarget-1)+iKFstate,2:end))),obj.plot.phatmarker,'color',obj.plot.phatcolor)
        plot(CLOCK.tvec,-2*sqrt(squeeze(obj.hist.Phat(nTgtState*(iTarget-1)+iKFstate,nTgtState*(iTarget-1)+iKFstate,2:end))),obj.plot.phatmarker,'color',obj.plot.phatcolor)
        
        
        xlabel('Time (secs)')
        ylabel(obj.plot.ylabeltarget(iKFstate));
        
        if iKFstate == 1
            legend([get(legend(gca),'string'),obj.plot.legend]);
        end
    end
    
end

% Agent Error Plot.
for iAgent = 1 : length(AGENT)
    % state errors:
    if strcmp(option,'central') % should be moidified! (to central)
        figure(SIM.nTarget+1+iAgent), hold on;
        suptitle(['Platform ',num2str(iAgent),' Bias Estimation Errors'])
    else % local estimation
        figure(SIM.nTarget+1+AGENT.id), hold on;
    end
    
    for iKFstate = 1 : nBiasState
        subplot(BiasList(iKFstate)), hold on;
        plot(CLOCK.tvec,obj.hist.xhat(nBiasState*(iAgent-1)+iKFstate,2:end)-AGENT(iAgent).SENSOR.bias(iKFstate),'marker',obj.plot.hbmarker,'color',obj.plot.hbcolor);
        plot(CLOCK.tvec,2*sqrt(squeeze(obj.hist.Phat(nBiasState*(iAgent-1)+iKFstate,nBiasState*(iAgent-1)+iKFstate,2:end))),obj.plot.phatmarker,'color',obj.plot.phatcolor)
        plot(CLOCK.tvec,-2*sqrt(squeeze(obj.hist.Phat(nBiasState*(iAgent-1)+iKFstate,nBiasState*(iAgent-1)+iKFstate,2:end))),obj.plot.phatmarker,'color',obj.plot.phatcolor)
        
        
        xlabel('Time (secs)')
        ylabel(obj.plot.ylabelbias(iKFstate));
        
        if iKFstate == 1
            legend([get(legend(gca),'string'),AGENT.ESTIMATOR.plot.legend]);
            
        end
    end
end

end