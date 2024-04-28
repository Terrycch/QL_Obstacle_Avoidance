function steer = simple_decision_TOA(scanValues)
if sum(scanValues(1:5)) > sum(scanValues(6:10))
    steer = -2;
else
    steer = 2;
end
end
