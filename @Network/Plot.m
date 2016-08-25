function obj = Plot( obj, CLOCK )
%PLOT plots network profile in terms of connection between agents

nAgent = length(obj.Z);

for iAgent = 1 : nAgent
    for jAgent = 1 : nAgent
        if iAgent ~= jAgent
            subplot(nAgent*100+10+iAgent),stairs(CLOCK.tvec,squeeze(obj.hist.graph(2:end,jAgent,iAgent))',...
                'LineWidth',obj.plot.lineWidth(jAgent),'color',obj.plot.color(jAgent,:)); hold on; % 1st : sender, 2nd : receiver
            legend([get(legend(gca),'string'),obj.plot.legend(jAgent)]);
            
            set(gca,'fontsize',12);
        end
    end
    
    if iAgent == nAgent
        xlabel(obj.plot.xlabel);
    end
    
    if jAgent == nAgent
        ylabel(obj.plot.ylabel(iAgent));
    end
    
    if iAgent == 1
        suptitle('Communication Status');
    end
    
    axis([0 CLOCK.tf -0.5 1.5]);
end


end

