
clear all;
close all;

load('boxplot.mat');

position_1 = 1:1:9;  
position_2 = 1.2:1:9.2;
position_3 = 1.4:1:9.4;

%% AGENT 1

figure(1)
box1 = boxplot([cost_dubins_inf.trace(1,:,1,1)',cost_dubins_inf.trace(1,:,1,2)',cost_dubins_inf.trace(1,:,1,3)',...
    cost_dubins_inf.trace(1,:,2,1)',cost_dubins_inf.trace(1,:,2,2)',cost_dubins_inf.trace(1,:,2,3)',...
    cost_dubins_inf.trace(1,:,4,1)',cost_dubins_inf.trace(1,:,4,2)',cost_dubins_inf.trace(1,:,4,3)'],...
    'colors','k','positions',position_1,'width',0.18);
title('Trace Comparison : Dubins Target Dynamics vs. Linear Dynamics Guess (Agent 1)');


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box2 = boxplot([cost_dubins_40.trace(1,:,1,1)',cost_dubins_40.trace(1,:,1,2)',cost_dubins_40.trace(1,:,1,3)',...
    cost_dubins_40.trace(1,:,2,1)',cost_dubins_40.trace(1,:,2,2)',cost_dubins_40.trace(1,:,2,3)',...
    cost_dubins_40.trace(1,:,4,1)',cost_dubins_40.trace(1,:,4,2)',cost_dubins_40.trace(1,:,4,3)'],...
    'colors','c','positions',position_2,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box3 = boxplot([cost_dubins_20.trace(1,:,1,1)',cost_dubins_20.trace(1,:,1,2)',cost_dubins_20.trace(1,:,1,3)',...
    cost_dubins_20.trace(1,:,2,1)',cost_dubins_20.trace(1,:,2,2)',cost_dubins_20.trace(1,:,2,3)',...
    cost_dubins_20.trace(1,:,4,1)',cost_dubins_20.trace(1,:,4,2)',cost_dubins_20.trace(1,:,4,3)'],...
    'colors','r','positions',position_3,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   


hold off   % Insert texts and labels 
ylabel('Trace') 
text('Position',[0.85,-0.03],'String','Target 1 Central') 
text('Position',[1.85,-0.03],'String','Target 1 MMNB') 
text('Position',[2.85,-0.03],'String','Target 1 Diag') 
text('Position',[3.85,-0.03],'String','Target 2 Central') 
text('Position',[4.85,-0.03],'String','Target 2 MMNB') 
text('Position',[5.85,-0.03],'String','Target 2 Diag') 
text('Position',[6.85,-0.03],'String','Bias Central') 
text('Position',[7.85,-0.03],'String','Bias MMNB') 
text('Position',[8.85,-0.03],'String','Bias Diag') 

set(gca,'fontsize',12)

ylim([0 0.8])

legend([box1(5,1),box2(5,1),box3(5,1)], {'Perfect Comm.','40m Comm. Range','20m Comm. Range'})


figure(2)
box1 = boxplot([cost_static_inf.trace(1,:,1,1)',cost_static_inf.trace(1,:,1,2)',cost_static_inf.trace(1,:,1,3)',...
    cost_static_inf.trace(1,:,2,1)',cost_static_inf.trace(1,:,2,2)',cost_static_inf.trace(1,:,2,3)',...
    cost_static_inf.trace(1,:,4,1)',cost_static_inf.trace(1,:,4,2)',cost_static_inf.trace(1,:,4,3)'],...
    'colors','k','positions',position_1,'width',0.18);
title('Trace Comparison : Static Target Dynamics vs. Linear Dynamics Guess (Agent 1)');


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box2 = boxplot([cost_static_40.trace(1,:,1,1)',cost_static_40.trace(1,:,1,2)',cost_static_40.trace(1,:,1,3)',...
    cost_static_40.trace(1,:,2,1)',cost_static_40.trace(1,:,2,2)',cost_static_40.trace(1,:,2,3)',...
    cost_static_40.trace(1,:,4,1)',cost_static_40.trace(1,:,4,2)',cost_static_40.trace(1,:,4,3)'],...
    'colors','c','positions',position_2,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box3 = boxplot([cost_static_20.trace(1,:,1,1)',cost_static_20.trace(1,:,1,2)',cost_static_20.trace(1,:,1,3)',...
    cost_static_20.trace(1,:,2,1)',cost_static_20.trace(1,:,2,2)',cost_static_20.trace(1,:,2,3)',...
    cost_static_20.trace(1,:,4,1)',cost_static_20.trace(1,:,4,2)',cost_static_20.trace(1,:,4,3)'],...
    'colors','r','positions',position_3,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   


hold off   % Insert texts and labels 
ylabel('Trace') 
text('Position',[0.85,-0.03],'String','Target 1 Central') 
text('Position',[1.85,-0.03],'String','Target 1 MMNB') 
text('Position',[2.85,-0.03],'String','Target 1 Diag') 
text('Position',[3.85,-0.03],'String','Target 2 Central') 
text('Position',[4.85,-0.03],'String','Target 2 MMNB') 
text('Position',[5.85,-0.03],'String','Target 2 Diag') 
text('Position',[6.85,-0.03],'String','Bias Central') 
text('Position',[7.85,-0.03],'String','Bias MMNB') 
text('Position',[8.85,-0.03],'String','Bias Diag') 

set(gca,'fontsize',12)

ylim([0 0.8])

legend([box1(5,1),box2(5,1),box3(5,1)], {'Perfect Comm.','40m Comm. Range','20m Comm. Range'})


%%

figure(3)
box1 = boxplot([cost_dubins_inf.mse(1,:,1,1)',cost_dubins_inf.mse(1,:,1,2)',cost_dubins_inf.mse(1,:,1,3)',...
    cost_dubins_inf.mse(1,:,2,1)',cost_dubins_inf.mse(1,:,2,2)',cost_dubins_inf.mse(1,:,2,3)',...
    cost_dubins_inf.mse(1,:,4,1)',cost_dubins_inf.mse(1,:,4,2)',cost_dubins_inf.mse(1,:,4,3)'],...
    'colors','k','positions',position_1,'width',0.18);
title('Norm of Position Error Comparison : Dubins Target Dynamics vs. Linear Dynamics Guess (Agent 1)');


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box2 = boxplot([cost_dubins_40.mse(1,:,1,1)',cost_dubins_40.mse(1,:,1,2)',cost_dubins_40.mse(1,:,1,3)',...
    cost_dubins_40.mse(1,:,2,1)',cost_dubins_40.mse(1,:,2,2)',cost_dubins_40.mse(1,:,2,3)',...
    cost_dubins_40.mse(1,:,4,1)',cost_dubins_40.mse(1,:,4,2)',cost_dubins_40.mse(1,:,4,3)'],...
    'colors','c','positions',position_2,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box3 = boxplot([cost_dubins_20.mse(1,:,1,1)',cost_dubins_20.mse(1,:,1,2)',cost_dubins_20.mse(1,:,1,3)',...
    cost_dubins_20.mse(1,:,2,1)',cost_dubins_20.mse(1,:,2,2)',cost_dubins_20.mse(1,:,2,3)',...
    cost_dubins_20.mse(1,:,4,1)',cost_dubins_20.mse(1,:,4,2)',cost_dubins_20.mse(1,:,4,3)'],...
    'colors','r','positions',position_3,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   


hold off   % Insert texts and labels 
ylabel('Norm of Position Error') 
text('Position',[0.85,-0.3],'String','Target 1 Central') 
text('Position',[1.85,-0.3],'String','Target 1 MMNB') 
text('Position',[2.85,-0.3],'String','Target 1 Diag') 
text('Position',[3.85,-0.3],'String','Target 2 Central') 
text('Position',[4.85,-0.3],'String','Target 2 MMNB') 
text('Position',[5.85,-0.3],'String','Target 2 Diag') 
text('Position',[6.85,-0.3],'String','Bias Central') 
text('Position',[7.85,-0.3],'String','Bias MMNB') 
text('Position',[8.85,-0.3],'String','Bias Diag') 

set(gca,'fontsize',12)

ylim([0 3])

legend([box1(5,1),box2(5,1),box3(5,1)], {'Perfect Comm.','40m Comm. Range','20m Comm. Range'})

figure(4)
box1 = boxplot([cost_static_inf.mse(1,:,1,1)',cost_static_inf.mse(1,:,1,2)',cost_static_inf.mse(1,:,1,3)',...
    cost_static_inf.mse(1,:,2,1)',cost_static_inf.mse(1,:,2,2)',cost_static_inf.mse(1,:,2,3)',...
    cost_static_inf.mse(1,:,4,1)',cost_static_inf.mse(1,:,4,2)',cost_static_inf.mse(1,:,4,3)'],...
    'colors','k','positions',position_1,'width',0.18);
title('Norm of Position Error Comparison : Static Target Dynamics vs. Linear Dynamics Guess (Agent 1)');


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box2 = boxplot([cost_static_40.mse(1,:,1,1)',cost_static_40.mse(1,:,1,2)',cost_static_40.mse(1,:,1,3)',...
    cost_static_40.mse(1,:,2,1)',cost_static_40.mse(1,:,2,2)',cost_static_40.mse(1,:,2,3)',...
    cost_static_40.mse(1,:,4,1)',cost_static_40.mse(1,:,4,2)',cost_static_40.mse(1,:,4,3)'],...
    'colors','c','positions',position_2,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box3 = boxplot([cost_static_20.mse(1,:,1,1)',cost_static_20.mse(1,:,1,2)',cost_static_20.mse(1,:,1,3)',...
    cost_static_20.mse(1,:,2,1)',cost_static_20.mse(1,:,2,2)',cost_static_20.mse(1,:,2,3)',...
    cost_static_20.mse(1,:,4,1)',cost_static_20.mse(1,:,4,2)',cost_static_20.mse(1,:,4,3)'],...
    'colors','r','positions',position_3,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   


hold off   % Insert texts and labels 
ylabel('Norm of Position Error') 
text('Position',[0.85,-0.3],'String','Target 1 Central') 
text('Position',[1.85,-0.3],'String','Target 1 MMNB') 
text('Position',[2.85,-0.3],'String','Target 1 Diag') 
text('Position',[3.85,-0.3],'String','Target 2 Central') 
text('Position',[4.85,-0.3],'String','Target 2 MMNB') 
text('Position',[5.85,-0.3],'String','Target 2 Diag') 
text('Position',[6.85,-0.3],'String','Bias Central') 
text('Position',[7.85,-0.3],'String','Bias MMNB') 
text('Position',[8.85,-0.3],'String','Bias Diag') 

set(gca,'fontsize',12)

ylim([0 3])

legend([box1(5,1),box2(5,1),box3(5,1)], {'Perfect Comm.','40m Comm. Range','20m Comm. Range'})

%% AGENT 2

figure(5)
box1 = boxplot([cost_dubins_inf.trace(2,:,1,1)',cost_dubins_inf.trace(2,:,1,2)',cost_dubins_inf.trace(2,:,1,3)',...
    cost_dubins_inf.trace(2,:,3,1)',cost_dubins_inf.trace(2,:,3,2)',cost_dubins_inf.trace(2,:,3,3)',...
    cost_dubins_inf.trace(2,:,4,1)',cost_dubins_inf.trace(2,:,4,2)',cost_dubins_inf.trace(2,:,4,3)'],...
    'colors','k','positions',position_1,'width',0.18);
title('Trace Comparison : Dubins Target Dynamics vs. Linear Dynamics Guess (Agent 2)');


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box2 = boxplot([cost_dubins_40.trace(2,:,1,1)',cost_dubins_40.trace(2,:,1,2)',cost_dubins_40.trace(2,:,1,3)',...
    cost_dubins_40.trace(2,:,3,1)',cost_dubins_40.trace(2,:,3,2)',cost_dubins_40.trace(2,:,3,3)',...
    cost_dubins_40.trace(2,:,4,1)',cost_dubins_40.trace(2,:,4,2)',cost_dubins_40.trace(2,:,4,3)'],...
    'colors','c','positions',position_2,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box3 = boxplot([cost_dubins_20.trace(2,:,1,1)',cost_dubins_20.trace(2,:,1,2)',cost_dubins_20.trace(2,:,1,3)',...
    cost_dubins_20.trace(2,:,3,1)',cost_dubins_20.trace(2,:,3,2)',cost_dubins_20.trace(2,:,3,3)',...
    cost_dubins_20.trace(2,:,4,1)',cost_dubins_20.trace(2,:,4,2)',cost_dubins_20.trace(2,:,4,3)'],...
    'colors','r','positions',position_3,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   


hold off   % Insert texts and labels 
ylabel('Trace') 
text('Position',[0.85,-0.05],'String','Target 1 Central') 
text('Position',[1.85,-0.05],'String','Target 1 MMNB') 
text('Position',[2.85,-0.05],'String','Target 1 Diag') 
text('Position',[3.85,-0.05],'String','Target 3 Central') 
text('Position',[4.85,-0.05],'String','Target 3 MMNB') 
text('Position',[5.85,-0.05],'String','Target 3 Diag') 
text('Position',[6.85,-0.05],'String','Bias Central') 
text('Position',[7.85,-0.05],'String','Bias MMNB') 
text('Position',[8.85,-0.05],'String','Bias Diag') 

set(gca,'fontsize',12)

ylim([0 1.5])

legend([box1(5,1),box2(5,1),box3(5,1)], {'Perfect Comm.','40m Comm. Range','20m Comm. Range'})


figure(6)
box1 = boxplot([cost_static_inf.trace(2,:,1,1)',cost_static_inf.trace(2,:,1,2)',cost_static_inf.trace(2,:,1,3)',...
    cost_static_inf.trace(2,:,3,1)',cost_static_inf.trace(2,:,3,2)',cost_static_inf.trace(2,:,3,3)',...
    cost_static_inf.trace(2,:,4,1)',cost_static_inf.trace(2,:,4,2)',cost_static_inf.trace(2,:,4,3)'],...
    'colors','k','positions',position_1,'width',0.18);
title('Trace Comparison : Static Target Dynamics vs. Linear Dynamics Guess (Agent 2)');


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box2 = boxplot([cost_static_40.trace(2,:,1,1)',cost_static_40.trace(2,:,1,2)',cost_static_40.trace(2,:,1,3)',...
    cost_static_40.trace(2,:,3,1)',cost_static_40.trace(2,:,3,2)',cost_static_40.trace(2,:,3,3)',...
    cost_static_40.trace(2,:,4,1)',cost_static_40.trace(2,:,4,2)',cost_static_40.trace(2,:,4,3)'],...
    'colors','c','positions',position_2,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box3 = boxplot([cost_static_20.trace(2,:,1,1)',cost_static_20.trace(2,:,1,2)',cost_static_20.trace(2,:,1,3)',...
    cost_static_20.trace(2,:,3,1)',cost_static_20.trace(2,:,3,2)',cost_static_20.trace(2,:,3,3)',...
    cost_static_20.trace(2,:,4,1)',cost_static_20.trace(2,:,4,2)',cost_static_20.trace(2,:,4,3)'],...
    'colors','r','positions',position_3,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   


hold off   % Insert texts and labels 
ylabel('Trace') 
text('Position',[0.85,-0.05],'String','Target 1 Central') 
text('Position',[1.85,-0.05],'String','Target 1 MMNB') 
text('Position',[2.85,-0.05],'String','Target 1 Diag') 
text('Position',[3.85,-0.05],'String','Target 3 Central') 
text('Position',[4.85,-0.05],'String','Target 3 MMNB') 
text('Position',[5.85,-0.05],'String','Target 3 Diag') 
text('Position',[6.85,-0.05],'String','Bias Central') 
text('Position',[7.85,-0.05],'String','Bias MMNB') 
text('Position',[8.85,-0.05],'String','Bias Diag') 

set(gca,'fontsize',12)

ylim([0 1.5])

legend([box1(5,1),box2(5,1),box3(5,1)], {'Perfect Comm.','40m Comm. Range','20m Comm. Range'})


%%

figure(7)
box1 = boxplot([cost_dubins_inf.mse(2,:,1,1)',cost_dubins_inf.mse(2,:,1,2)',cost_dubins_inf.mse(2,:,1,3)',...
    cost_dubins_inf.mse(2,:,3,1)',cost_dubins_inf.mse(2,:,3,2)',cost_dubins_inf.mse(2,:,3,3)',...
    cost_dubins_inf.mse(2,:,4,1)',cost_dubins_inf.mse(2,:,4,2)',cost_dubins_inf.mse(2,:,4,3)'],...
    'colors','k','positions',position_1,'width',0.18);
title('Norm of Position Error Comparison : Dubins Target Dynamics vs. Linear Dynamics Guess (Agent 2)');


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box2 = boxplot([cost_dubins_40.mse(2,:,1,1)',cost_dubins_40.mse(2,:,1,2)',cost_dubins_40.mse(2,:,1,3)',...
    cost_dubins_40.mse(2,:,3,1)',cost_dubins_40.mse(2,:,3,2)',cost_dubins_40.mse(2,:,3,3)',...
    cost_dubins_40.mse(2,:,4,1)',cost_dubins_40.mse(2,:,4,2)',cost_dubins_40.mse(2,:,4,3)'],...
    'colors','c','positions',position_2,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box3 = boxplot([cost_dubins_20.mse(2,:,1,1)',cost_dubins_20.mse(2,:,1,2)',cost_dubins_20.mse(2,:,1,3)',...
    cost_dubins_20.mse(2,:,3,1)',cost_dubins_20.mse(2,:,3,2)',cost_dubins_20.mse(2,:,3,3)',...
    cost_dubins_20.mse(2,:,4,1)',cost_dubins_20.mse(2,:,4,2)',cost_dubins_20.mse(2,:,4,3)'],...
    'colors','r','positions',position_3,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   


hold off   % Insert texts and labels 
ylabel('Norm of Position Error') 
text('Position',[0.85,-0.3],'String','Target 1 Central') 
text('Position',[1.85,-0.3],'String','Target 1 MMNB') 
text('Position',[2.85,-0.3],'String','Target 1 Diag') 
text('Position',[3.85,-0.3],'String','Target 3 Central') 
text('Position',[4.85,-0.3],'String','Target 3 MMNB') 
text('Position',[5.85,-0.3],'String','Target 3 Diag') 
text('Position',[6.85,-0.3],'String','Bias Central') 
text('Position',[7.85,-0.3],'String','Bias MMNB') 
text('Position',[8.85,-0.3],'String','Bias Diag') 

set(gca,'fontsize',12)

ylim([0 3])

legend([box1(5,1),box2(5,1),box3(5,1)], {'Perfect Comm.','40m Comm. Range','20m Comm. Range'})


figure(8)
box1 = boxplot([cost_static_inf.mse(2,:,1,1)',cost_static_inf.mse(2,:,1,2)',cost_static_inf.mse(2,:,1,3)',...
    cost_static_inf.mse(2,:,3,1)',cost_static_inf.mse(2,:,3,2)',cost_static_inf.mse(2,:,3,3)',...
    cost_static_inf.mse(2,:,4,1)',cost_static_inf.mse(2,:,4,2)',cost_static_inf.mse(2,:,4,3)'],...
    'colors','k','positions',position_1,'width',0.18);
title('Norm of Position Error Comparison : Static Target Dynamics vs. Linear Dynamics Guess (Agent 2)');


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box2 = boxplot([cost_static_40.mse(2,:,1,1)',cost_static_40.mse(2,:,1,2)',cost_static_40.mse(2,:,1,3)',...
    cost_static_40.mse(2,:,3,1)',cost_static_40.mse(2,:,3,2)',cost_static_40.mse(2,:,3,3)',...
    cost_static_40.mse(2,:,4,1)',cost_static_40.mse(2,:,4,2)',cost_static_40.mse(2,:,4,3)'],...
    'colors','c','positions',position_2,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box3 = boxplot([cost_static_20.mse(2,:,1,1)',cost_static_20.mse(2,:,1,2)',cost_static_20.mse(2,:,1,3)',...
    cost_static_20.mse(2,:,3,1)',cost_static_20.mse(2,:,3,2)',cost_static_20.mse(2,:,3,3)',...
    cost_static_20.mse(2,:,4,1)',cost_static_20.mse(2,:,4,2)',cost_static_20.mse(2,:,4,3)'],...
    'colors','r','positions',position_3,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   


hold off   % Insert texts and labels 
ylabel('Norm of Position Error') 
text('Position',[0.85,-0.3],'String','Target 1 Central') 
text('Position',[1.85,-0.3],'String','Target 1 MMNB') 
text('Position',[2.85,-0.3],'String','Target 1 Diag') 
text('Position',[3.85,-0.3],'String','Target 3 Central') 
text('Position',[4.85,-0.3],'String','Target 3 MMNB') 
text('Position',[5.85,-0.3],'String','Target 3 Diag') 
text('Position',[6.85,-0.3],'String','Bias Central') 
text('Position',[7.85,-0.3],'String','Bias MMNB') 
text('Position',[8.85,-0.3],'String','Bias Diag') 

set(gca,'fontsize',12)

ylim([0 3])

legend([box1(5,1),box2(5,1),box3(5,1)], {'Perfect Comm.','40m Comm. Range','20m Comm. Range'})

%% AGENT 3

figure(9)
box1 = boxplot([cost_dubins_inf.trace(3,:,2,1)',cost_dubins_inf.trace(3,:,2,2)',cost_dubins_inf.trace(3,:,2,3)',...
    cost_dubins_inf.trace(3,:,3,1)',cost_dubins_inf.trace(3,:,3,2)',cost_dubins_inf.trace(3,:,3,3)',...
    cost_dubins_inf.trace(3,:,4,1)',cost_dubins_inf.trace(3,:,4,2)',cost_dubins_inf.trace(3,:,4,3)'],...
    'colors','k','positions',position_1,'width',0.18);
title('Trace Comparison : Dubins Target Dynamics vs. Linear Dynamics Guess (Agent 3)');


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box2 = boxplot([cost_dubins_40.trace(3,:,2,1)',cost_dubins_40.trace(3,:,2,2)',cost_dubins_40.trace(3,:,2,3)',...
    cost_dubins_40.trace(3,:,3,1)',cost_dubins_40.trace(3,:,3,2)',cost_dubins_40.trace(3,:,3,3)',...
    cost_dubins_40.trace(3,:,4,1)',cost_dubins_40.trace(3,:,4,2)',cost_dubins_40.trace(3,:,4,3)'],...
    'colors','c','positions',position_2,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box3 = boxplot([cost_dubins_20.trace(3,:,2,1)',cost_dubins_20.trace(3,:,2,2)',cost_dubins_20.trace(3,:,2,3)',...
    cost_dubins_20.trace(3,:,3,1)',cost_dubins_20.trace(3,:,3,2)',cost_dubins_20.trace(3,:,3,3)',...
    cost_dubins_20.trace(3,:,4,1)',cost_dubins_20.trace(3,:,4,2)',cost_dubins_20.trace(3,:,4,3)'],...
    'colors','r','positions',position_3,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   


hold off   % Insert texts and labels 
ylabel('Trace') 
text('Position',[0.85,-0.05],'String','Target 2 Central') 
text('Position',[1.85,-0.05],'String','Target 2 MMNB') 
text('Position',[2.85,-0.05],'String','Target 2 Diag') 
text('Position',[3.85,-0.05],'String','Target 3 Central') 
text('Position',[4.85,-0.05],'String','Target 3 MMNB') 
text('Position',[5.85,-0.05],'String','Target 3 Diag') 
text('Position',[6.85,-0.05],'String','Bias Central') 
text('Position',[7.85,-0.05],'String','Bias MMNB') 
text('Position',[8.85,-0.05],'String','Bias Diag') 

set(gca,'fontsize',12)

ylim([0 1.5])

legend([box1(5,1),box2(5,1),box3(5,1)], {'Perfect Comm.','40m Comm. Range','20m Comm. Range'})


figure(10)
box1 = boxplot([cost_static_inf.trace(3,:,2,1)',cost_static_inf.trace(3,:,2,2)',cost_static_inf.trace(3,:,2,3)',...
    cost_static_inf.trace(3,:,3,1)',cost_static_inf.trace(3,:,3,2)',cost_static_inf.trace(3,:,3,3)',...
    cost_static_inf.trace(3,:,4,1)',cost_static_inf.trace(3,:,4,2)',cost_static_inf.trace(3,:,4,3)'],...
    'colors','k','positions',position_1,'width',0.18);
title('Trace Comparison : Static Target Dynamics vs. Linear Dynamics Guess (Agent 3)');


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box2 = boxplot([cost_static_40.trace(3,:,2,1)',cost_static_40.trace(3,:,2,2)',cost_static_40.trace(3,:,2,3)',...
    cost_static_40.trace(3,:,3,1)',cost_static_40.trace(3,:,3,2)',cost_static_40.trace(3,:,3,3)',...
    cost_static_40.trace(3,:,4,1)',cost_static_40.trace(3,:,4,2)',cost_static_40.trace(3,:,4,3)'],...
    'colors','c','positions',position_2,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box3 = boxplot([cost_static_20.trace(3,:,2,1)',cost_static_20.trace(3,:,2,2)',cost_static_20.trace(3,:,2,3)',...
    cost_static_20.trace(3,:,3,1)',cost_static_20.trace(3,:,3,2)',cost_static_20.trace(3,:,3,3)',...
    cost_static_20.trace(3,:,4,1)',cost_static_20.trace(3,:,4,2)',cost_static_20.trace(3,:,4,3)'],...
    'colors','r','positions',position_3,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   


hold off   % Insert texts and labels 
ylabel('Trace') 
text('Position',[0.85,-0.05],'String','Target 2 Central') 
text('Position',[1.85,-0.05],'String','Target 2 MMNB') 
text('Position',[2.85,-0.05],'String','Target 2 Diag') 
text('Position',[3.85,-0.05],'String','Target 3 Central') 
text('Position',[4.85,-0.05],'String','Target 3 MMNB') 
text('Position',[5.85,-0.05],'String','Target 3 Diag') 
text('Position',[6.85,-0.05],'String','Bias Central') 
text('Position',[7.85,-0.05],'String','Bias MMNB') 
text('Position',[8.85,-0.05],'String','Bias Diag') 

set(gca,'fontsize',12)

ylim([0 1.5])

legend([box1(5,1),box2(5,1),box3(5,1)], {'Perfect Comm.','40m Comm. Range','20m Comm. Range'})


%%

figure(11)
box1 = boxplot([cost_dubins_inf.mse(3,:,2,1)',cost_dubins_inf.mse(3,:,2,2)',cost_dubins_inf.mse(3,:,2,3)',...
    cost_dubins_inf.mse(3,:,3,1)',cost_dubins_inf.mse(3,:,3,2)',cost_dubins_inf.mse(3,:,3,3)',...
    cost_dubins_inf.mse(3,:,4,1)',cost_dubins_inf.mse(3,:,4,2)',cost_dubins_inf.mse(3,:,4,3)'],...
    'colors','k','positions',position_1,'width',0.18);
title('Norm of Position Error Comparison : Dubins Target Dynamics vs. Linear Dynamics Guess (Agent 3)');


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box2 = boxplot([cost_dubins_40.mse(3,:,2,1)',cost_dubins_40.mse(3,:,2,2)',cost_dubins_40.mse(3,:,2,3)',...
    cost_dubins_40.mse(3,:,3,1)',cost_dubins_40.mse(3,:,3,2)',cost_dubins_40.mse(3,:,3,3)',...
    cost_dubins_40.mse(3,:,4,1)',cost_dubins_40.mse(3,:,4,2)',cost_dubins_40.mse(3,:,4,3)'],...
    'colors','c','positions',position_2,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box3 = boxplot([cost_dubins_20.mse(3,:,2,1)',cost_dubins_20.mse(3,:,2,2)',cost_dubins_20.mse(3,:,2,3)',...
    cost_dubins_20.mse(3,:,3,1)',cost_dubins_20.mse(3,:,3,2)',cost_dubins_20.mse(3,:,3,3)',...
    cost_dubins_20.mse(3,:,4,1)',cost_dubins_20.mse(3,:,4,2)',cost_dubins_20.mse(3,:,4,3)'],...
    'colors','r','positions',position_3,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   


hold off   % Insert texts and labels 
ylabel('Norm of Position Error') 
text('Position',[0.85,-0.3],'String','Target 2 Central') 
text('Position',[1.85,-0.3],'String','Target 2 MMNB') 
text('Position',[2.85,-0.3],'String','Target 2 Diag') 
text('Position',[3.85,-0.3],'String','Target 3 Central') 
text('Position',[4.85,-0.3],'String','Target 3 MMNB') 
text('Position',[5.85,-0.3],'String','Target 3 Diag') 
text('Position',[6.85,-0.3],'String','Bias Central') 
text('Position',[7.85,-0.3],'String','Bias MMNB') 
text('Position',[8.85,-0.3],'String','Bias Diag') 

set(gca,'fontsize',12)

ylim([0 3])

legend([box1(5,1),box2(5,1),box3(5,1)], {'Perfect Comm.','40m Comm. Range','20m Comm. Range'})


figure(12)
box1 = boxplot([cost_static_inf.mse(3,:,2,1)',cost_static_inf.mse(3,:,2,2)',cost_static_inf.mse(3,:,2,3)',...
    cost_static_inf.mse(3,:,3,1)',cost_static_inf.mse(3,:,3,2)',cost_static_inf.mse(3,:,3,3)',...
    cost_static_inf.mse(3,:,4,1)',cost_static_inf.mse(3,:,4,2)',cost_static_inf.mse(3,:,4,3)'],...
    'colors','k','positions',position_1,'width',0.18);
title('Norm of Position Error Comparison : Static Target Dynamics vs. Linear Dynamics Guess (Agent 3)');


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box2 = boxplot([cost_static_40.mse(3,:,2,1)',cost_static_40.mse(3,:,2,2)',cost_static_40.mse(3,:,2,3)',...
    cost_static_40.mse(3,:,3,1)',cost_static_40.mse(3,:,3,2)',cost_static_40.mse(3,:,3,3)',...
    cost_static_40.mse(3,:,4,1)',cost_static_40.mse(3,:,4,2)',cost_static_40.mse(3,:,4,3)'],...
    'colors','c','positions',position_2,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   

box3 = boxplot([cost_static_20.mse(3,:,2,1)',cost_static_20.mse(3,:,2,2)',cost_static_20.mse(3,:,2,3)',...
    cost_static_20.mse(3,:,3,1)',cost_static_20.mse(3,:,3,2)',cost_static_20.mse(3,:,3,3)',...
    cost_static_20.mse(3,:,4,1)',cost_static_20.mse(3,:,4,2)',cost_static_20.mse(3,:,4,3)'],...
    'colors','r','positions',position_3,'width',0.18);


set(gca,'XTickLabel',{' '})  % Erase xlabels   
hold on  % Keep the Month_O boxplots on figure overlap the Month_S boxplots   


hold off   % Insert texts and labels 
ylabel('Norm of Position Error') 
text('Position',[0.85,-0.3],'String','Target 2 Central') 
text('Position',[1.85,-0.3],'String','Target 2 MMNB') 
text('Position',[2.85,-0.3],'String','Target 2 Diag') 
text('Position',[3.85,-0.3],'String','Target 3 Central') 
text('Position',[4.85,-0.3],'String','Target 3 MMNB') 
text('Position',[5.85,-0.3],'String','Target 3 Diag') 
text('Position',[6.85,-0.3],'String','Bias Central') 
text('Position',[7.85,-0.3],'String','Bias MMNB') 
text('Position',[8.85,-0.3],'String','Bias Diag') 

set(gca,'fontsize',12)

ylim([0 3])

legend([box1(5,1),box2(5,1),box3(5,1)], {'Perfect Comm.','40m Comm. Range','20m Comm. Range'})

