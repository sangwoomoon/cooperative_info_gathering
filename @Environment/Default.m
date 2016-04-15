function o = Default (o, CLOCK)

    o.xlength = [-100 100]; % meters
    o.ylength = [-100 100]; % meters
    
    o.bound = [ o.xlength(1), o.ylength(1);    
                o.xlength(1), o.ylength(2);
                o.xlength(2), o.ylength(2);
                o.xlength(2), o.ylength(1)];
    
end