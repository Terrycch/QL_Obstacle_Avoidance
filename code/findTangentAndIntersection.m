function intersectionPoints = findTangentAndIntersection(point, circleCenter, circleRadius)
    % 计算出障碍物被雷达扫描的一条直线
    % 计算点与圆心之间的连线
    theta = atan((circleCenter(2) - point(2))/(circleCenter(1) - point(1)));
    % 计算l3与圆的交点
    distance = sqrt((point(1)-circleCenter(1))^2+(point(2)-circleCenter(2))^2);
    factor = circleRadius / distance;
    meetpoint = [circleCenter(1) - factor*(circleCenter(1)-point(1)), circleCenter(2) - factor*(circleCenter(2)-point(2))];
    factor1 = (1-factor)*circleRadius;
    intersectionPoints = [meetpoint(1) - factor1*sin(theta),meetpoint(2) + factor1*cos(theta);
                          meetpoint(1) + factor1*sin(theta),meetpoint(2) - factor1*cos(theta)];
end
