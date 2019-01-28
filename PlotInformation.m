function PlotInformation(planner,flagSensor,flagComm,jSim)

% plot the result 
figure(100*jSim + 2*(planner.id-1)+1)
plot(planner.gaussAll.hist.time,planner.gaussAll.hist.H,'g-','Linewidth',2); hold on;
plot(planner.pmAll.hist.time,planner.pmAll.hist.H,'r-','Linewidth',2); 
if flagComm
    plot(planner.pmSample.hist.time,planner.pmSample.hist.H,'m-','Linewidth',2);
    plot(planner.pmSeparate.hist.time,planner.pmSeparate.hist.H,'c-','Linewidth',2);
    plot(planner.gaussRtilde.hist.time,planner.gaussRtilde.hist.H,'b-','linewidth',2);
end

switch flagSensor
    case 'PosLinear'
        if flagComm
            legend('true','PM: all events','PM: sampled comm','PM: separate comm','modified cov matrix');
        else
            legend('true','PM: all events');            
        end
    otherwise
        if flagComm
            legend('Gaussian: all events','PM: all events','PM: sampled comm','PM: separate comm','Gaussian: modified cov matrix');
        else
            legend('Gaussian: all events','PM: all events'); 
        end
end
xlabel('t (receding horizon time step)');
ylabel('entropy');

figure(100*jSim + 2*planner.id)
plot(planner.gaussAll.I,'g-','Linewidth',2); hold on;
plot(planner.pmAll.I,'r-','Linewidth',2);
if flagComm
    plot(planner.pmSample.I,'m-','Linewidth',2);
    plot(planner.pmSeparate.I,'c-','Linewidth',2);
    plot(planner.gaussRtilde.I,'b-','linewidth',2);
end

switch flagSensor
    case 'PosLinear'
        if flagComm
            legend('true','PM: all events','PM: sampled comm','PM: separate comm','Gaussian: modified cov matrix');
        else
            legend('PM: all events','true');            
        end
    otherwise
        if flagComm
            legend('Gaussian: all events','PM: all events','PM: sampled comm','PM: separate comm','Gaussian: modified cov matrix');
        else
            legend('Gaussian: all events','PM: all events'); 
        end
end
xlabel('t (receding horizon time step)');
ylabel('I(X_{k:k+t};Y_{k:k+t})');


end