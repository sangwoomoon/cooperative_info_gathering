function o = Default (o, CLOCK, size, delta)

    o.xlength = [size(1) size(2)]; % meters
    o.ylength = [size(3) size(4)]; % meters
    
    o.bound = [ o.xlength(1), o.ylength(1);    
                o.xlength(1), o.ylength(2);
                o.xlength(2), o.ylength(2);
                o.xlength(2), o.ylength(1)];
            
    o.bound = [o.bound; o.bound(1,:)];
            
    o.delta = delta;
    
    [o.x, o.y] = meshgrid(o.xlength(1):o.delta:o.xlength(2),o.ylength(1):o.delta:o.ylength(2));
    
    axis([o.xlength,o.ylength]);
    axis equal;
    
    boundary = [o.xlength(1),o.ylength(1);
            o.xlength(1),o.ylength(2);
            o.xlength(2),o.ylength(2);
            o.xlength(2),o.ylength(1);
            o.xlength(1),o.ylength(1)];
        
        % environment boundary plot
        o.plot.h.field = patch(boundary(:,1),boundary(:,2),'c');
        alpha(0.1);
        
        % detection point plot
        o.plot.h.field = plot(o.x,o.y,'k*');
    
end