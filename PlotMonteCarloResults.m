function PlotMonteCarloResults(sim,mSim,nSim,flagCondition)


% initialize array
pmAllCost = nan(mSim,nSim);
pmSampleCost = nan(mSim,nSim);
subLabel = nan(1,mSim);

if sim(1,1).flagComm
    pmSeparateCost = nan(mSim,nSim);
    gaussRtildeCost = nan(mSim,nSim);
    gaussAllCost = nan(mSim,nSim);
    
    pmAllTime = nan(mSim,nSim);
    pmSampleTime = nan(mSim,nSim);
end


switch sim(1,1).flagSensor
    
    case 'PosLinear' % compare with true value (true is known)
   
        % initialization for percentage RMS error data 
        if sim(1,1).flagComm
            costErrRMS = nan(4,mSim);
        else
            costErrRMS = nan(1,mSim);
        end
        
        % retrieve necessary data and plot results
        for jSim = 1:mSim
            for iSim = 1:nSim
                pmAllCost(jSim,iSim) = sum(sim(jSim,iSim).planner(1).pmAll.I);
                gaussAllCost(jSim,iSim) = sum(sim(jSim,iSim).planner(1).gaussAll.I);
                if sim(jSim,iSim).flagComm
                    pmSeparateCost(jSim,iSim) = sum(sim(jSim,iSim).planner(1).pmSeparate.I);
                    gaussRtildeCost(jSim,iSim) = sum(sim(jSim,iSim).planner(1).gaussRtilde.I);
                    pmSampleCost(jSim,iSim) = sum(sim(jSim,iSim).planner(1).pmSample.I);
                end
            end
            
            % take root-mean-squared error from gaussAllCost (exact solution when
            % linear gaussian): RMS error percentage
            costErrRMS(1,jSim) = rms((pmAllCost(jSim,:) - gaussAllCost(jSim,:))./gaussAllCost(jSim,:))*100;
            if sim(jSim,iSim).flagComm
                costErrRMS(2,jSim) = rms((pmSampleCost(jSim,:) - gaussAllCost(jSim,:))./gaussAllCost(jSim,:))*100;
                costErrRMS(3,jSim) = rms((pmSeparateCost(jSim,:) - gaussAllCost(jSim,:))./gaussAllCost(jSim,:))*100;
                costErrRMS(4,jSim) = rms((gaussRtildeCost(jSim,:) - gaussAllCost(jSim,:))./gaussAllCost(jSim,:))*100;
            end
            
            % label
            switch flagCondition
                case 'nPt'
                    subLabel(jSim) = sim(jSim,1).planner(1).PTset.nPt;
                    subTitleInfo(1) = sim(jSim,1).planner(1).param.clock.nT; % receding horizon
                    if sim(1,1).flagComm
                        subTitleInfo(2) = ComputeCommProb(sim(jSim,1).agent(1).s,sim(jSim,1).agent(2).s); % beta
                    else
                        subTitleInfo(2) = sim(jSim,1).planner(1).param.pdf.dRefPt; % dRefPt
                    end
                    label{jSim} = num2str(subLabel(jSim));
                case 'nT'
                    subLabel(jSim) = sim(jSim,1).planner(1).param.clock.nT;
                    subTitleInfo(1) = sim(jSim,1).planner(1).PTset(1).nPt; % particle number
                    if sim(1,1).flagComm
                        subTitleInfo(2) = ComputeCommProb(sim(jSim,1).agent(1).s,sim(jSim,1).agent(2).s); % beta
                    else
                        subTitleInfo(2) = sim(jSim,1).planner(1).param.pdf.dRefPt; % dRefPt
                    end
                    label{jSim} = num2str(subLabel(jSim));
                case 'dRefPt'
                    subLabel(jSim) = sim(jSim,1).planner(1).param.pdf.dRefPt;
                    subTitleInfo(1) = sim(jSim,1).planner(1).param.clock.nT; % receding horizon
                    subTitleInfo(2) = sim(jSim,1).planner(1).PTset(1).nPt; % particle number
                    label{jSim} = num2str(subLabel(jSim));
                case 'dist'
                    subLabel(jSim) = ComputeCommProb(sim(jSim,1).agent(1).s,sim(jSim,1).agent(2).s);
                    subTitleInfo(1) = sim(jSim,1).planner(1).param.clock.nT; % receding horizon
                    subTitleInfo(2) = sim(jSim,1).planner(1).PTset(1).nPt; % particle number
                    label{jSim} = num2str(subLabel(jSim),'%1.3f');
            end
        end
        
        figure(200),
        bar(costErrRMS');
        set(gca,'xticklabel',label)
        switch flagCondition
            case 'nPt'
                xlabel('number of particles (N_p)');
                if sim(1,1).flagComm
                    title(['T=',num2str(subTitleInfo(1)),', \beta=',num2str(subTitleInfo(2))]);
                else
                    title(['T=',num2str(subTitleInfo(1)),', \Deltax^q=',num2str(subTitleInfo(2))]);
                end
            case 'nT'
                xlabel('number of receding horizon (T)');
                if sim(1,1).flagComm
                    title(['N_p=',num2str(subTitleInfo(1)),', \beta=',num2str(subTitleInfo(2))]);
                else
                    title(['N_p=',num2str(subTitleInfo(1)),', \Deltax^q=',num2str(subTitleInfo(2))]);                    
                end
            case 'dist'
                xlabel('probability of delivery (\beta)');
                title(['T=',num2str(subTitleInfo(1)),', N_p=',num2str(subTitleInfo(2))]);
            case 'dRefPt'
                xlabel('length of cell of discretized domain [m]');
                title(['T=',num2str(subTitleInfo(1)),', N_p=',num2str(subTitleInfo(2))]);
        end
        
        ylabel('percentage of RMS error of I(X_{k+1:k+T};Z_{k+1:k+T})');
        if sim(jSim,iSim).flagComm
            legend('PM: \Sigma P(B)I(X;\zeta)','PM: I(X;\zeta_{sample})','PM: P(B)I(X;Y)','Gaussian: R/\beta');
        end
        
        
        
    case 'range_bear' % we don't know true value, so just make comparison
        
        % initialization for percentage RMS error data
        if sim(1,1).flagComm
            costRMS = nan(4,mSim);
        else
            costRMS = nan(1,mSim);
        end
        
        % retrieve necessary data and plot results
        for jSim = 1:mSim
            for iSim = 1:nSim
                pmAllCost(jSim,iSim) = sim(jSim,iSim).planner(1).pmAll.I(end);
                gaussAllCost(jSim,iSim) = sim(jSim,iSim).planner(1).gaussAll.I(end);
                if sim(jSim,iSim).flagComm
                    pmSeparateCost(jSim,iSim) = sim(jSim,iSim).planner(1).pmSeparate.I(end);
                    gaussRtildeCost(jSim,iSim) = sim(jSim,iSim).planner(1).gaussRtilde.I(end);
                    pmSampleCost(jSim,iSim) = sim(jSim,iSim).planner(1).pmSample.I(end);
                end
            end
            
            % just make comparison because we don't know the exact solution
            % compare RMS values
            costRMS(1,jSim) = rms(pmAllCost(jSim,:));
            if ~sim(jSim,iSim).flagComm
                costRMS(2,jSim) = rms(gaussAllCost(jSim,:));
            else
                costRMS(2,jSim) = rms(pmSampleCost(jSim,:));
                costRMS(3,jSim) = rms(pmSeparateCost(jSim,:));
                costRMS(4,jSim) = rms(gaussAllCost(jSim,:));
                costRMS(5,jSim) = rms(gaussRtildeCost(jSim,:));
            end
            
            switch flagCondition
                case 'nPt'
                    subLabel(jSim) = sim(jSim,1).planner(1).PTset.nPt;
                    subTitleInfo(1) = sim(jSim,1).planner(1).param.clock.nT; % receding horizon
                    if sim(1,1).flagComm
                        subTitleInfo(2) = ComputeCommProb(sim(jSim,1).agent(1).s,sim(jSim,1).agent(2).s); % beta
                    else
                        subTitleInfo(2) = sim(jSim,1).planner(1).param.pdf.dRefPt; % dRefPt
                    end
                    label{jSim} = num2str(subLabel(jSim));
                case 'nT'
                    subLabel(jSim) = sim(jSim,1).planner(1).param.clock.nT;
                    subTitleInfo(1) = sim(jSim,1).planner(1).PTset(1).nPt; % particle number
                    if sim(1,1).flagComm
                        subTitleInfo(2) = ComputeCommProb(sim(jSim,1).agent(1).s,sim(jSim,1).agent(2).s); % beta
                    else
                        subTitleInfo(2) = sim(jSim,1).planner(1).param.pdf.dRefPt; % dRefPt
                    end
                    label{jSim} = num2str(subLabel(jSim));
                case 'dRefPt'
                    subLabel(jSim) = sim(jSim,1).planner(1).param.pdf.dRefPt;
                    subTitleInfo(1) = sim(jSim,1).planner(1).param.clock.nT; % receding horizon
                    subTitleInfo(2) = sim(jSim,1).planner(1).PTset(1).nPt; % particle number
                    label{jSim} = num2str(subLabel(jSim));
                case 'dist'
                    subLabel(jSim) = ComputeCommProb(sim(jSim,1).agent(1).s,sim(jSim,1).agent(2).s);
                    subTitleInfo(1) = sim(jSim,1).planner(1).param.clock.nT; % receding horizon
                    subTitleInfo(2) = sim(jSim,1).planner(1).PTset(1).nPt; % particle number
                    label{jSim} = num2str(subLabel(jSim),'%1.3f');
            end
        end
        
        figure(200),
        bar(costRMS');
        set(gca,'xticklabel',label)
        switch flagCondition
            case 'nPt'
                xlabel('number of particles (N_p)');
                if sim(1,1).flagComm
                    title(['T=',num2str(subTitleInfo(1)),', \beta=',num2str(subTitleInfo(2))]);
                else
                    title(['T=',num2str(subTitleInfo(1)),', \Deltax^q=',num2str(subTitleInfo(2))]);
                end
            case 'nT'
                xlabel('number of receding horizon (T)');
                if sim(1,1).flagComm
                    title(['N_p=',num2str(subTitleInfo(1)),', \beta=',num2str(subTitleInfo(2))]);
                else
                    title(['N_p=',num2str(subTitleInfo(1)),', \Deltax^q=',num2str(subTitleInfo(2))]);                    
                end
            case 'dist'
                xlabel('probability of delivery (\beta)');
                title(['T=',num2str(subTitleInfo(1)),', N_p=',num2str(subTitleInfo(2))]);
            case 'dRefPt'
                xlabel('length of cell of discretized domain [m]');
                title(['T=',num2str(subTitleInfo(1)),', N_p=',num2str(subTitleInfo(2))]);
        end
        
        ylabel('RMS of I(X_{k+1:k+T};Z_{k+1:k+T})');
        if ~sim(jSim,iSim).flagComm
            legend('PM: \Sigma I(X;Y)','Gaussian: \Sigma I(X;Y)');
        else
            legend('PM: \Sigma P(B)I(X;\zeta)','PM: I(X;\zeta_{sample})','PM: P(B)I(X;Y)','Gaussian: \Sigma I(X;Y)','Gaussian: R/\beta');
        end
        
end

% compute average computation time and display in command window

for jSim = 1:mSim
    for iSim = 1:nSim
        pmAllTime(jSim,iSim) = sim(jSim,iSim).planner(1).pmAll.time;
        if sim(jSim,iSim).flagComm
            pmSampleTime(jSim,iSim) = sim(jSim,iSim).planner(1).pmSample.time;
        end
    end
    
    pmAllTimeAvg = mean(pmAllTime,2);
    if sim(jSim,iSim).flagComm
        pmSampleTimeAvg = mean(pmSampleTime,2);
        fprintf('%s = %d\t Exact: %3.3f\t Sampled: %3.3f\n', flagCondition, subLabel(jSim), pmAllTimeAvg(jSim), pmSampleTimeAvg(jSim));
    else
        fprintf('%s = %d\t Exact: %3.3f\n', flagCondition, subLabel(jSim), pmAllTimeAvg(jSim));
    end
end
    



end