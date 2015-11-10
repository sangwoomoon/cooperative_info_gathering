function o = Plot (o, AGENT, TARGET, CLOCK, SIMULATION, option)

tl=sum(TARGET(1).bKFx);  % number of state for targets concerned with KF process
al=sum(AGENT(1).bKFs);   % number of state for agents concerned with KF process

AgentStateList = [];
AgentStateList1 = [];
AgentStateList2 = [];

TargetStateList = [];
TargetStateList1 = [];
TargetStateList2 = [];

iKFplotidx = 0;
iKFidxp = [];
for iAgentState = 1 : length(AGENT(1).s)
    if AGENT(1).bKFs(iAgentState) == 1
        iKFplotidx = iKFplotidx + 1;
        AgentStateList1 = [AgentStateList1, al*100+20+2*(iKFplotidx-1)+1];
        AgentStateList2 = [AgentStateList2, al*100+20+2*iKFplotidx];
        iKFidxp = [iKFidxp,iAgentState];
    end
end
AgentStateList = [AgentStateList1, AgentStateList2];

iKFplotidx = 0;
iKFidxt = [];
for iTargetState = 1 : length(TARGET(1).x)
    if TARGET(1).bKFx(iTargetState) == 1
        iKFplotidx = iKFplotidx + 1;
        TargetStateList1 = [TargetStateList1, tl*100+20+2*(iKFplotidx-1)+1];
        TargetStateList2 = [TargetStateList2, tl*100+20+2*iKFplotidx];
        iKFidxt = [iKFidxt,iTargetState];
    end
end
TargetStateList = [TargetStateList1, TargetStateList2];

% Target Error Plot
for iTarget = 1 : length(TARGET)
    % Target bias errors:
    figure(iTarget+1), hold on;
    if strcmp(option,'central')
        suptitle(['Target ',num2str(iTarget), ' State Estimation Errors'])
    end
    
    for iKFstate = 1 : 2*tl
        if iKFstate < tl+1 % actual
            subplot(TargetStateList(iKFstate)), hold on;
            plot(CLOCK.tvec,o.hist.Xhat(tl*(iTarget-1)+iKFstate,:)-TARGET(iTarget).hist.x(iKFidxt(iKFstate),2:end),'marker',o.plot.htmarker,'color',o.plot.htcolor);
            plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+iKFstate,tl*(iTarget-1)+iKFstate,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
            plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+iKFstate,tl*(iTarget-1)+iKFstate,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
        else % percentage
            subplot(TargetStateList(iKFstate)), hold on;
            plot(CLOCK.tvec,...
                (o.hist.Xhat(tl*(iTarget-1)+iKFstate-tl,:)-TARGET(iTarget).hist.x(iKFidxt(iKFstate-tl),2:end))...
                ./TARGET(iTarget).hist.x(iKFidxt(iKFstate-tl),2:end)*100,...
                'marker',o.plot.htmarker,'color',o.plot.htcolor);
        end
    
        xlabel('Time (secs)')
        ylabel(SIMULATION.CENTRAL_KF.plot.ylabeltarget(iKFstate));
        
        if iKFstate == 1
            switch option
                case 'central' % Centralized Case
                    legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
                case 'local' % local case
                    legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
                case 'decentral' % decentralized case
                    legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
                    
            end
        end
    end
end


% Agent Error Plot.
for iAgent = 1 : length(AGENT)
    % state errors:
    if strcmp(option,'central') % Centralized Case
        figure(length(TARGET)+1+iAgent), hold on;
        suptitle(['Platform ',num2str(iAgent),' Bias Estimation Errors'])
        idx = iAgent;
    else % local/decentralized Case
        figure(length(TARGET)+1+AGENT.id), hold on;
        if strcmp(option,'local')
            idx = iAgent;
        elseif strcmp(option,'decentral')
            idx = iAgent;
        end
    end
    
    for iKFstate = 1 : 2*al
        if iKFstate < al + 1
            subplot(AgentStateList(iKFstate)), hold on
            plot(CLOCK.tvec,o.hist.Xhat(length(TARGET)*tl+al*(idx-1)+iKFstate,:)-AGENT(iAgent).hist.s(iKFidxp(iKFstate),2:end),'marker',o.plot.hpmarker,'color',o.plot.hpcolor);
            plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(length(TARGET)*tl+al*(idx-1)+iKFstate,length(TARGET)*tl+al*(iAgent-1)+iKFstate,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
            plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(length(TARGET)*tl+al*(idx-1)+iKFstate,length(TARGET)*tl+al*(iAgent-1)+iKFstate,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
        else
            subplot(AgentStateList(iKFstate)), hold on
            plot(CLOCK.tvec,...
                (o.hist.Xhat(length(TARGET)*tl+al*(idx-1)+iKFstate-al,:)-AGENT(iAgent).hist.s(iKFidxp(iKFstate-al),2:end))...
                ./AGENT(iAgent).hist.s(iKFidxp(iKFstate-al),2:end)*100,...
                'marker',o.plot.hpmarker,'color',o.plot.hpcolor);
            
        end
        xlabel('Time (secs)')
        ylabel(SIMULATION.CENTRAL_KF.plot.ylabelagent(iKFstate));
        
        if iKFstate == 1
            switch option
                case 'central' % Centralized Case
                    legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
                case 'local' % local case
                    legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
                case 'decentral' % decentralized case
                    legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
                    
            end
        end
    end
end

end