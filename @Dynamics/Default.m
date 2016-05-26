function o = Default( o, CONTROL, CLOCK, id, option )
    
    o.x = [];
    switch (option)
        case ('Agent')
            fprintf('Agent %d dynamics is not assigned : should be modelized by users!\n', id)
        case ('Target')
            fprintf('Agent %d dynamics is not assigned : should be modelized by users!\n', id)
    end

end

