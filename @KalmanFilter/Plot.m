function obj = Plot (obj, AGENT, TARGET, CLOCK, SIM, option)

% when guessed sensor model is bias
for iTarget = 1 : length(obj.TARGET)
    
    for iAgent = 1 : length(AGENT)
        switch (obj.SENSOR{iAgent}.spec)
            case ('RelCartBias')
                nBiasState(iAgent)=length(obj.SENSOR{iAgent}.bias);
            otherwise
                nBiasState(iAgent) = 0;
        end
        
        nTgtState(iTarget)=obj.TARGET(iTarget).DYNAMICS.nState;
        
        TargetList{iTarget} = [];
        
        % make index of subplot : target state
        for iTgtState = 1 : nTgtState
            TargetList{iTarget} = [TargetList{iTarget}, nTgtState(iTarget)*100+10+iTgtState];
        end
    end
    
end


% Target Error Plot
ptTarget = 0;

for iTarget = 1 : length(TARGET)
    
    % Target state errors:
    figure(iTarget+SIM.iFigure), hold on;
    if strcmp(option,'central')
        suptitle(['Target ',num2str(iTarget), ' State Estimation Errors'])
        bPlot = ones(1,SIM.nTarget); % for all targets
    else % local
        bPlot = AGENT(iAgent).SENSOR.bTrack; % for tracking targets
    end
    
    if bPlot(iTarget) == 1 % only for tracking targets
        
        ptTarget = ptTarget + 1;
        
        for iKFstate = 1 : nTgtState(ptTarget)
            subplot(TargetList{ptTarget}(iKFstate)), hold on;
            %plot(CLOCK.tvec,obj.hist.xhat(nTgtState*(iTarget-1)+iKFstate+nBiasState,2:end)-TARGET(iTarget).DYNAMICS.hist.x(iKFstate,2:end),'marker',obj.plot.htmarker,'color',obj.plot.htcolor);
            if rem(iKFstate,2) == 1 % ad-hoc, should be changed! (coordinate conversion function is required!)
                plot(CLOCK.tvec,obj.hist.xhat(nTgtState(ptTarget)*(ptTarget-1)+iKFstate+sum(nBiasState),2:end)-TARGET(iTarget).DYNAMICS.hist.pos((iKFstate==1)*1+(iKFstate==3)*2,2:end),'marker',obj.plot.htmarker,'color',obj.plot.htcolor);
            else
                plot(CLOCK.tvec,obj.hist.xhat(nTgtState(ptTarget)*(ptTarget-1)+iKFstate+sum(nBiasState),2:end)-TARGET(iTarget).DYNAMICS.hist.vel((iKFstate==2)*1+(iKFstate==4)*2,2:end),'marker',obj.plot.htmarker,'color',obj.plot.htcolor);
            end
            plot(CLOCK.tvec,2*sqrt(squeeze(obj.hist.Phat(nTgtState(ptTarget)*(ptTarget-1)+iKFstate+sum(nBiasState),nTgtState(ptTarget)*(ptTarget-1)+iKFstate+sum(nBiasState),2:end))),obj.plot.phatmarker,'color',obj.plot.phatcolor)
            plot(CLOCK.tvec,-2*sqrt(squeeze(obj.hist.Phat(nTgtState(ptTarget)*(ptTarget-1)+iKFstate+sum(nBiasState),nTgtState(ptTarget)*(ptTarget-1)+iKFstate+sum(nBiasState),2:end))),obj.plot.phatmarker,'color',obj.plot.phatcolor)
            
            if strcmp(option,'central')
                xlabel('Time (secs)')
                ylabel(obj.plot.ylabeltarget(iKFstate));
            end
            
            if iKFstate == 1
                legend([get(legend(gca),'string'),obj.plot.legend]);
            end
        end
        
    end
    
end

% Agent Error Plot. (only for bias)

for iAgent = 1 : length(AGENT)

    % make index of subplot : bias state
    BiasList{iAgent} = [];

    for iBiasState = 1 : nBiasState(iAgent)
        BiasList{iAgent} = [BiasList{iAgent}, nBiasState(iAgent)*100+10+iBiasState];
    end


    switch (obj.SENSOR{iAgent}.spec)
        
        case ('RelCartBias')
                        
            % state errors:
            if strcmp(option,'central') % should be moidified! (to central)
                figure(SIM.nTarget+SIM.iFigure+iAgent), hold on;
                suptitle(['Platform ',num2str(iAgent),' Bias Estimation Errors'])
            else
                figure(SIM.nTarget+SIM.iFigure+AGENT(iAgent).id), hold on;
            end
            
            for iKFstate = 1 : nBiasState(iAgent)
                subplot(BiasList{iAgent}(iKFstate)), hold on;
                
                switch (AGENT(iAgent).SENSOR.spec)
                    case ('RelCartBias')
                        bias = AGENT(iAgent).SENSOR.bias(iKFstate);     
                    otherwise
                        bias = 0;
                end
                
                plot(CLOCK.tvec,obj.hist.xhat(nBiasState(iAgent)*(iAgent-1)+iKFstate,2:end)-bias,'marker',obj.plot.hbmarker,'color',obj.plot.hbcolor);
                plot(CLOCK.tvec,2*sqrt(squeeze(obj.hist.Phat(nBiasState(iAgent)*(iAgent-1)+iKFstate,nBiasState(iAgent)*(iAgent-1)+iKFstate,2:end))),obj.plot.phatmarker,'color',obj.plot.phatcolor)
                plot(CLOCK.tvec,-2*sqrt(squeeze(obj.hist.Phat(nBiasState(iAgent)*(iAgent-1)+iKFstate,nBiasState(iAgent)*(iAgent-1)+iKFstate,2:end))),obj.plot.phatmarker,'color',obj.plot.phatcolor)
                
                if strcmp(option,'central')
                    xlabel('Time (secs)')
                    ylabel(obj.plot.ylabelbias(iKFstate));
                end
                
                if iKFstate == 1
                    legend([get(legend(gca),'string'),obj.plot.legend]);
                    
                end
            end
            
        otherwise
            
    end
end


end