
% this script is to illustrate how the grid-based entropy computation works
clear
close all;

domain = [-20 20];
refPt = domain(1):0.1:domain(2);
refPtGrid = domain(1):1:domain(2);

pt = (domain(1)+5)*ones(1,10) + (domain(2)-domain(1)-10)*rand(1,10);
w = ones(1,10)./length(pt);
Q = 3;

for iPt = 1:length(pt)
   
    % 1. substract mean from domain
    ptDomainDiff = refPt - pt(iPt);

    % 2. compute exponential part
    ptDomainExpSuper = -1/2*(1/Q*ptDomainDiff.^2);

    % 3. compute remain parts. the results is pdf.
    pdf(iPt,:) = (1/sqrt(2*pi*Q)).*exp(ptDomainExpSuper);
    
    figure(1)
    plot(refPt,pdf(iPt,:),'k--','linewidth',2); hold on;
    
end

mixedPdf = sum(pdf,1);
mixedPdf = mixedPdf./2;

plot(refPt,mixedPdf,'r-','linewidth',3);
plot(pt,zeros(1,10),'ko','linewidth',6);

approxIdx = 5:10:401;
approxPdf = mixedPdf(approxIdx);
approxPdf(end+1) = mixedPdf(end);

figure(2)
plot(refPt,mixedPdf,'r-','linewidth',3); hold on;
stairs(refPtGrid,approxPdf,'linewidth',3,'color','m')
stem(refPtGrid,approxPdf,'LineStyle','--','marker','none','color','m')


y = [-15,15];
R = 15;

% 1. substract mean from domain
pdf = [];

for iY = 1:length(y)
ptDomainDiff = refPt - y(iY);

% 2. compute exponential part
ptDomainExpSuper = -1/2*(1/R*ptDomainDiff.^2);

% 3. compute remain parts. the results is pdf.
pdf(iY,:) = (1/sqrt(2*pi*R)).*exp(ptDomainExpSuper);
end

likePdf = sum(pdf,1)/3;


measPdf = likePdf.*mixedPdf;

figure(3)
plot(refPt,mixedPdf,'r-','linewidth',3); hold on;
plot(refPt,likePdf.*5,'g--','linewidth',3);
plot(refPt,measPdf.*20,'b-','linewidth',3);

approxIdx = 5:10:401;
approxPdf = measPdf(approxIdx);
approxPdf(end+1) = measPdf(end);

legend('P(X_{t}|Z_{t-1})','P(Z_{t}|X_{t})','P(X_{t}|Z_{t})');


figure(4)
plot(refPt,measPdf.*20,'b-','linewidth',3); hold on;
stairs(refPtGrid,approxPdf.*20,'linewidth',3,'color','m')
stem(refPtGrid,approxPdf.*20,'LineStyle','--','marker','none','color','m')
axis([domain,0,0.4])
