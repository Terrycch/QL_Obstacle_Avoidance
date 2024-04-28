function [trajectory] = generateTrajectory(sampletime, type)
if type == "circle"
    time = 100;
    startpoint.x = 2.5;
    startpoint.y = 0.5;
    startpoint.theta = 0;
    total_time = time/sampletime;
    for index =1:total_time
        trajectory(index).w = (2*pi)/(time);
        trajectory(index).v = trajectory(index).w * 2;
    end
    trajectory(1).x = startpoint.x;
    trajectory(1).y = startpoint.y;
    trajectory(1).theta = startpoint.theta;
    for index = 2:total_time
        trajectory(index).theta = trajectory(index-1).theta + sampletime * trajectory(index-1).w;
        %
        if(trajectory(index).theta > (pi))
            trajectory(index).theta = trajectory(index).theta - 2*pi;
        elseif(trajectory(index).theta < -(pi))
            trajectory(index).theta = trajectory(index).theta + 2*pi;
        end
        %
        delta_l = sampletime * trajectory(index-1).v;
        trajectory(index).x = trajectory(index-1).x + cos(trajectory(index).theta) * delta_l;
        trajectory(index).y = trajectory(index-1).y + sin(trajectory(index).theta) * delta_l;
    end
end
end