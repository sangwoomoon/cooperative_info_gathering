function o = Plot (o, AGENT, TARGET, CLOCK, SIMULATION, option)

tl=length(TARGET(1).x);  % number of state for targets
% al=length(AGENT(1).s);   % number of state for agents


% Target Error Plot
for iTarget = 1 : length(TARGET)
    % Target bias errors:
    figure(2*(iTarget-1)+1+1), hold on;
    if strcmp(option,'central')
        suptitle(['Target ',num2str(iTarget), ' Bias Estimation Errors'])
    end
    subplot(411), hold on;
    o.plot.ht1{iTarget} = plot(CLOCK.tvec,o.hist.Xhat(tl*(iTarget-1)+1,:)-TARGET(iTarget).hist.x(1,2:end),'marker',o.plot.htmarker,'color',o.plot.htcolor);
    plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+1,tl*(iTarget-1)+1,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+1,tl*(iTarget-1)+1,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    
    switch option
        case 'central' % Centralized Case
            xlabel('Time (secs)')
            ylabel('Target Easting Bias error (m)')
            legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
        case 'local' % local case
            legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
        case 'decentral' % decentralized case
            legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
    end
    
    %%%
    subplot(412), hold on;
    o.plot.ht2{iTarget} = plot(CLOCK.tvec,o.hist.Xhat(tl*iTarget,:)-TARGET(iTarget).hist.x(2,2:end),'marker',o.plot.htmarker,'color',o.plot.htcolor);
    plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+2,tl*(iTarget-1)+2,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+2,tl*(iTarget-1)+2,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    
    switch option
        case 'central' % Centralized Case
            xlabel('Time (secs)')
            ylabel('Target Northing Bias error (m)')
%             legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
        case 'local' % local case
%             legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
        case 'decentral' % decentralized case
%             legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
    end
    
    subplot(413), hold on;
    o.plot.ht1{iTarget} = plot(CLOCK.tvec,o.hist.Xhat(tl*(iTarget-1)+3,:)-TARGET(iTarget).hist.x(3,2:end),'marker',o.plot.htmarker,'color',o.plot.htcolor);
    plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+3,tl*(iTarget-1)+3,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+3,tl*(iTarget-1)+3,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    
    switch option
        case 'central' % Centralized Case
            xlabel('Time (secs)')
            ylabel('Target Easting Bias error (m)')
%             legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
        case 'local' % local case
%             legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
        case 'decentral' % decentralized case
%             legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
    end
    
    %%%
    subplot(414), hold on;
    o.plot.ht2{iTarget} = plot(CLOCK.tvec,o.hist.Xhat(tl*(iTarget-1)+4,:)-TARGET(iTarget).hist.x(4,2:end),'marker',o.plot.htmarker,'color',o.plot.htcolor);
    plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+4,tl*(iTarget-1)+4,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+4,tl*(iTarget-1)+4,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    
    switch option
        case 'central' % Centralized Case
            xlabel('Time (secs)')
            ylabel('Target Northing Bias error (m)')
%             legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
        case 'local' % local case
%             legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
        case 'decentral' % decentralized case
%             legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
    end
    
    figure(2*(iTarget-1)+2+1), hold on;
    if strcmp(option,'central')
        suptitle(['Target ',num2str(iTarget), ' State Estimation Errors'])
    end
    subplot(411), hold on;
    o.plot.ht1{iTarget} = plot(CLOCK.tvec,o.hist.Xhat(tl*(iTarget-1)+5,:)-TARGET(iTarget).hist.x(5,2:end),'marker',o.plot.htmarker,'color',o.plot.htcolor);
    plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+5,tl*(iTarget-1)+5,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+5,tl*(iTarget-1)+5,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    
    switch option
        case 'central' % Centralized Case
            xlabel('Time (secs)')
            ylabel('Target Easting error (m)')
            legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
        case 'local' % local case
            legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
        case 'decentral' % decentralized case
            legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
    end
    
    %%%
    subplot(412), hold on;
    o.plot.ht2{iTarget} = plot(CLOCK.tvec,o.hist.Xhat(tl*(iTarget-1)+6,:)-TARGET(iTarget).hist.x(6,2:end),'marker',o.plot.htmarker,'color',o.plot.htcolor);
    plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+6,tl*(iTarget-1)+6,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+6,tl*(iTarget-1)+6,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    
    switch option
        case 'central' % Centralized Case
            xlabel('Time (secs)')
            ylabel('Target Easting Velocity error (m/s)')
%             legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
        case 'local' % local case
%             legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
        case 'decentral' % decentralized case
%             legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
    end
    subplot(413), hold on;
    o.plot.ht1{iTarget} = plot(CLOCK.tvec,o.hist.Xhat(tl*(iTarget-1)+7,:)-TARGET(iTarget).hist.x(7,2:end),'marker',o.plot.htmarker,'color',o.plot.htcolor);
    plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+7,tl*(iTarget-1)+7,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+7,tl*(iTarget-1)+7,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    
    switch option
        case 'central' % Centralized Case
            xlabel('Time (secs)')
            ylabel('Target Northing error (m)')
%             legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
        case 'local' % local case
%             legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
        case 'decentral' % decentralized case
%             legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
    end
    
    %%%
    subplot(414), hold on;
    o.plot.ht2{iTarget} = plot(CLOCK.tvec,o.hist.Xhat(tl*(iTarget-1)+8,:)-TARGET(iTarget).hist.x(8,2:end),'marker',o.plot.htmarker,'color',o.plot.htcolor);
    plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+8,tl*(iTarget-1)+8,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(tl*(iTarget-1)+8,tl*(iTarget-1)+8,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
    
    switch option
        case 'central' % Centralized Case
            xlabel('Time (secs)')
            ylabel('Target Northing Velocity error (m/s)')
%             legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
        case 'local' % local case
%             legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
        case 'decentral' % decentralized case
%             legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
    end
end

% % Agent Error Plot.
% for iAgent = 1 : length(AGENT)
%     % state errors:
%     if strcmp(option,'central') % Centralized Case
%         figure(length(TARGET)+1+iAgent), hold on;
%         suptitle(['Platform ',num2str(iAgent),' State Estimation Errors'])
%         idx = iAgent;
%     else % local/decentralized Case
%         figure(length(TARGET)+1+AGENT.id), hold on;
%         if strcmp(option,'local')
%             idx = iAgent;
%         elseif strcmp(option,'decentral')
%             idx = AGENT(iAgent).id;
%         end
%     end
%     subplot(411), hold on
%     o.plot.hp1{iAgent}= plot(CLOCK.tvec,o.hist.Xhat(length(TARGET)*tl+al*(idx-1)+1,:)-AGENT(iAgent).hist.s(1,2:end),'marker',o.plot.hpmarker,'color',o.plot.hpcolor);
% %    plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(length(TARGET)*tl+al*(idx-1)+1,length(TARGET)*tl+al*(iAgent-1)+1,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
% %    plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(length(TARGET)*tl+al*(idx-1)+1,length(TARGET)*tl+al*(iAgent-1)+1,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
%     
%     switch option
%         case 'central' % Centralized Case
%             xlabel('Time (secs)')
%             ylabel('Agent Easting error (m)')
%             legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
%         case 'local' % local case
%             legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
%         case 'decentral' % decentralized case
%             legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
%     end
%     
%     %%%
%     subplot(412), hold on
%     o.plot.hp2{iAgent}=plot(CLOCK.tvec,o.hist.Xhat(length(TARGET)*tl+al*(idx-1)+2,:)-AGENT(iAgent).hist.s(2,2:end),'marker',o.plot.hpmarker,'color',o.plot.hpcolor);
% %    plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(length(TARGET)*tl+al*(idx-1)+2,length(TARGET)*tl+al*(iAgent-1)+2,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
% %    plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(length(TARGET)*tl+al*(idx-1)+2,length(TARGET)*tl+al*(iAgent-1)+2,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
% 
%     switch option
%         case 'central' % Centralized Case
%             xlabel('Time (secs)')
%             ylabel('Agent Easting Velocity error (m)')
% %             legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
%         case 'local' % local case
% %             legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
%         case 'decentral' % decentralized case
% %             legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
%     end
%     
%     %%%
%     subplot(413), hold on
%     o.plot.hp3{iAgent}=plot(CLOCK.tvec,o.hist.Xhat(length(TARGET)*tl+al*(idx-1)+3,:)-AGENT(iAgent).hist.s(3,2:end),'marker',o.plot.hpmarker,'color',o.plot.hpcolor);
% %    plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(length(TARGET)*tl+al*(idx-1)+3,length(TARGET)*tl+al*(iAgent-1)+3,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
% %    plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(length(TARGET)*tl+al*(idx-1)+3,length(TARGET)*tl+al*(iAgent-1)+3,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
%     
%     switch option
%         case 'central' % Centralized Case
%             xlabel('Time (secs)')
%             ylabel('Agent Northing error (m)')
% %             legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
%         case 'local' % local case
% %             legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
%         case 'decentral' % decentralized case
% %             legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
%     end
%     
%     %%%
%     subplot(414), hold on
%     o.plot.hp4{iAgent}=plot(CLOCK.tvec,o.hist.Xhat(length(TARGET)*tl+al*(idx-1)+4,:)-AGENT(iAgent).hist.s(4,2:end),'marker',o.plot.hpmarker,'color',o.plot.hpcolor);
% %    plot(CLOCK.tvec,2*sqrt(squeeze(o.hist.Phat(length(TARGET)*tl+al*(idx-1)+4,length(TARGET)*tl+al*(iAgent-1)+4,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
% %    plot(CLOCK.tvec,-2*sqrt(squeeze(o.hist.Phat(length(TARGET)*tl+al*(idx-1)+4,length(TARGET)*tl+al*(iAgent-1)+4,2:end))),o.plot.phatmarker,'color',o.plot.phatcolor)
%     
%     switch option
%         case 'central' % Centralized Case
%             xlabel('Time (secs)')
%             ylabel('Agent Northing Velocity error (m)')
% %             legend([get(legend(gca),'string'),SIMULATION.CENTRAL_KF.plot.legend]);
%         case 'local' % local case
% %             legend([get(legend(gca),'string'),AGENT.LOCAL_KF.plot.legend]);
%         case 'decentral' % decentralized case
% %             legend([get(legend(gca),'string'),AGENT.DECEN_KF.plot.legend]);
%     end
% end

end