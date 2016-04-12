function o = Plot (o, AGENT, TARGET, CLOCK, SIMULATION, option)

tl=length(TARGET(1).x);  % number of state for targets concerned with KF process
TargetStateList = [];

% iKFplotidx = 0;
% iKFidxp = [];
% for iAgentState = 1 : length(AGENT(1).s)
%     if AGENT(1).bKFs(iAgentState) == 1
%         iKFplotidx = iKFplotidx + 1;
%         AgentStateList1 = [AgentStateList1, al*100+20+2*(iKFplotidx-1)+1];
%         AgentStateList2 = [AgentStateList2, al*100+20+2*iKFplotidx];
%         iKFidxp = [iKFidxp,iAgentState];
%     end
% end
% AgentStateList = [AgentStateList1, AgentStateList2];

iKFplotidx = 0;
iKFidxt = [];
for iTargetState = 1 : length(TARGET(1).x)
%     if TARGET(1).bKFx(iTargetState) == 1
        iKFplotidx = iKFplotidx + 1;
        TargetStateList = [TargetStateList, tl*100+10+iKFplotidx];
        iKFidxt = [iKFidxt,iTargetState];
%     end
end


% Target Error Plot
for iTarget = 1 : length(TARGET)
    
    % Target state errors:
    figure(iTarget+1), hold on;
%     if strcmp(option,'central')
        suptitle(['Target ',num2str(iTarget), ' State Estimation Errors'])
%     end
    
    if TARGET(iTarget).bLandMark == 0 % if the target is a landmark, the plot should be stopped
        for iKFstate = 1 : tl
            subplot(TargetStateList(iKFstate)), hold on;
            plot(CLOCK.tvec,o.hist.Xhat(tl*(iTarget-1)+iKFstate,2:end)-TARGET(iTarget).hist.x(iKFidxt(iKFstate),2:end),'marker',o.plot.htmarker,'color',o.plot.htcolor);
            plot(CLOCK.tvec,3*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+iKFstate,tl*(iTarget-1)+iKFstate,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
            plot(CLOCK.tvec,-3*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+iKFstate,tl*(iTarget-1)+iKFstate,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
            
            xlabel('Time (secs)')
            ylabel(AGENT(1).LOCAL_KF.plot.ylabeltarget(iKFstate));
            
            if iKFstate == 1
                switch option
                    case 'central' % Centralized Case
                        legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
                    case 'local' % local case
                        legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
                    case 'decentral' % decentralized case
                        legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
                    case 'fDDF' % fDDF based local case
                        legend([get(legend(gca),'string'),AGENT.FDDF_KF.plot.legend]);
                        
                end
            end
        end
    end
end

% % Agent Error Plot.
% for iAgent = 1 : length(AGENT)
%     % state errors:
%     if strcmp(option,'central') % Centralized Case
%         figure((length(TARGET)-SIMULATION.nLandMark)+1+iAgent), hold on;
%         suptitle(['Platform ',num2str(iAgent),' Bias Estimation Errors'])
%         idx = iAgent;
%     else % local/decentralized Case
%         figure((length(TARGET)-SIMULATION.nLandMark)+1+AGENT.id), hold on;
%         idx = iAgent;
%     end
%     
%     for iKFstate = 1 : 2*al
%         if iKFstate < al + 1
%             subplot(AgentStateList(iKFstate)), hold on
%             plot(CLOCK.tvec,o.hist.Xhat((length(TARGET)-SIMULATION.nLandMark)*tl+al*(idx-1)+iKFstate,2:end)-AGENT(iAgent).hist.s(iKFidxp(iKFstate),2:end),'marker',o.plot.hpmarker,'color',o.plot.hpcolor);
%             plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat((length(TARGET)-SIMULATION.nLandMark)*tl+al*(idx-1)+iKFstate,(length(TARGET)-SIMULATION.nLandMark)*tl+al*(iAgent-1)+iKFstate,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
%             plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat((length(TARGET)-SIMULATION.nLandMark)*tl+al*(idx-1)+iKFstate,(length(TARGET)-SIMULATION.nLandMark)*tl+al*(iAgent-1)+iKFstate,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
%         else
%             subplot(AgentStateList(iKFstate)), hold on
%             plot(CLOCK.tvec,...
%                 (o.hist.Xhat((length(TARGET)-SIMULATION.nLandMark)*tl+al*(idx-1)+iKFstate-al,2:end)-AGENT(iAgent).hist.s(iKFidxp(iKFstate-al),2:end))...
%                 ./((abs(o.hist.Xhat((length(TARGET)-SIMULATION.nLandMark)*tl+al*(idx-1)+iKFstate-al,2:end))+abs(AGENT(iAgent).hist.s(iKFidxp(iKFstate-al),2:end)))./2)*100,...
%                 'marker',o.plot.hpmarker,'color',o.plot.hpcolor);
%             
%         end
%         xlabel('Time (secs)')
%         ylabel(SIMULATION.CENTRAL_KF.plot.ylabelagent(iKFstate));
%         
%         if iKFstate == 1
%             switch option
%                 case 'central' % Centralized Case
%                     legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
%                 case 'local' % local case
%                     legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
%                 case 'decentral' % decentralized case
%                     legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
%                 case 'fDDF' % fDDF based local case
%                     legend([get(legend(gca),'string'),AGENT.FDDF_KF.plot.legend]);
%                     
%             end
%         end
%     end
% end

end