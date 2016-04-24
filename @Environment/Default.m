function o = Default (o, CLOCK)

    o.xlength = [-300 300]; % meters
    o.ylength = [-300 300]; % meters
    
    o.kr = 0.2;
    
    o.bound = [ o.xlength(1), o.ylength(1);    
                o.xlength(1), o.ylength(2);
                o.xlength(2), o.ylength(2);
                o.xlength(2), o.ylength(1)];
    
end