function PlotPDF(pdf,pt,pdfParam,option)

nState = length(pt(:,1));
nPt = length(pt(1,:));

switch nState
    case 1
        plot(pdfParam.refPt,pdf,'b-','LineWidth',2); hold on;
        plot(pt,zeros(1,nPt),'m.','LineWidth',3);
    case 2
        switch option
            case 'uniform'
                contourf(squeeze(pdfParam.refPt(1,:,:)),squeeze(pdfParam.refPt(2,:,:)),squeeze(pdf)); hold on;
                colormap(pink);
                plot3(pt(1,:),pt(2,:),ones(1,nPt),'m.','LineWidth',3);
                axis equal;
            case 'cylinder'
                % contourf(pt(1,:),pt(2,:),pdf); hold on;
                % colormap(pink);
                plot(pt(1,:),pt(2,:),'m.','LineWidth',3);
                axis equal;
            case 'voronoi'
                % contourf(pt(1,:),pt(2,:),pdf); hold on;
                % colormap(pink);
                plot(pt(1,:),pt(2,:),'m.','LineWidth',3);
                axis equal;                
        end
                
end

end