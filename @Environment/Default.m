function o = Default (o, CLOCK, size)

    o.xlength = [size(1) size(2)]; % meters
    o.ylength = [size(3) size(4)]; % meters
    
    o.bound = [ o.xlength(1), o.ylength(1);    
                o.xlength(1), o.ylength(2);
                o.xlength(2), o.ylength(2);
                o.xlength(2), o.ylength(1)];
    
end