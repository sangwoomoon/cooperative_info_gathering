function PlotMonteCarloResults(sim,mSim,nSim,flagCondition)


% initialize array
pmAllCost = nan(mSim,nSim);
pmSampleCost = nan(mSim,nSim);
subtitle = nan(1,mSim);

if sim(1,1).flagComm
    gaussSeparateCost = nan(mSim,nSim);
    gaussRtildeCost = nan(mSim,nSim);
    gaussAllCost = nan(mSim,nSim);
    
    costErrRMS = nan(4,mSim);
else
    costErrRMS = nan(1,mSim); 
end

    switch sim(1,1).flagSensor
        case 'PosLinear' % compare with true value (true is known)
            
            % retrieve necessary data and plot results
            for jSim = 1:mSim
                for iSim = 1:nSim
                    pmAllCost(jSim,iSim) = sim(jSim,iSim).planner(1).pmAll.I(end);
                    gaussAllCost(jSim,iSim) = sim(jSim,iSim).planner(1).gaussAll.I(end);
                    if sim(jSim,iSim).flagComm
                        gaussSeparateCost(jSim,iSim) = sim(jSim,iSim).planner(1).pmSeparate.I(end);
                        gaussRtildeCost(jSim,iSim) = sim(jSim,iSim).planner(1).gaussRtilde.I(end);
                        pmSampleCost(jSim,iSim) = sim(jSim,iSim).planner(1).pmSample.I(end);
                    end
                end
                
                % take root-mean-squared error from gaussAllCost (exact solution when
                % linear gaussian)
                costErrRMS(1,jSim) = rms(pmAllCost(jSim,:) - gaussAllCost(jSim,:));
                if sim(jSim,iSim).flagComm
                    costErrRMS(2,jSim) = rms(pmSampleCost(jSim,:) - gaussAllCost(jSim,:));
                    costErrRMS(3,jSim) = rms(gaussSeparateCost(jSim,:) - gaussAllCost(jSim,:));
                    costErrRMS(4,jSim) = rms(gaussRtildeCost(jSim,:) - gaussAllCost(jSim,:));
                end
                
                switch flagCondition
                    case 'nPt'
                        subtitle(jSim) = sim(jSim,1).planner(1).PTset.nPt;
                        
                    case 'nT'
                        subtitle(jSim) = sim(jSim,1).planner(1).param.clock.nT;
                end
                label{jSim} = num2str(subtitle(jSim));
            end
            
            figure(200),
            bar(costErrRMS');
            set(gca,'xticklabel',label)
            switch flagCondition
                case 'nPt'
                    xlabel('Number of particles (N_p)');
                case 'nT'
                    xlabel('Number of receding horizon (T)');
                case 'dist'
                    xlabel('distance of two agents [m]');
            end
            
            ylabel('RMS error of I(X_{k+1:k+T};Z_{k+1:k+T})');
            if sim(jSim,iSim).flagComm
                legend('PM: all events','PM: sampled comm','PM: separate comm','modified cov matrix');
            end
            
        case 'range_bear' % we don't know true value, so just make comparison
            
            
    end
    
end