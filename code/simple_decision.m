function steer = simple_decision(scanValues)
    steer = -1 * [0.8 , 0.8, 0.8, 0.8, 1] * [scanValues(1) - scanValues(10);scanValues(2) - scanValues(9);scanValues(3) - scanValues(8);scanValues(4) - scanValues(7);scanValues(5) - scanValues(6);];
    steer = approximate(steer);
end

function outvalue = approximate(invalue)
if invalue <= 0.5 && invalue >= -0.5
    outvalue = 0;
elseif invalue > 0
    if invalue <= 1.5
        outvalue = 1;
    else
        outvalue =2;
    end
else
    if invalue >= -1.5
        outvalue = -1;
    else
        outvalue = -2;
    end
end
end
