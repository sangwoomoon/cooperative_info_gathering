
clear;

range = [0 700 0 700];
dr = 1;

[x,y] = meshgrid(range(1):dr:range(2),range(3):dr:range(4));
beta = max(0.5*erfc((sqrt(x.^2+y.^2)-461)/195)-0.085,0);

surf(x,y,beta,'edgecolor','none'), axis equal, xlabel('East [m]'), ylabel('North [m]')
view(0,90)
colormap(flipud(jet));
colorbar