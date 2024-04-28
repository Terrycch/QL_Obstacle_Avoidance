function intersectionPoint = findLineSegmentIntersection(p1, p2, q1, q2)
    % 计算线段的方向向量
    v1 = p2 - p1;
    v2 = q2 - q1;

    % 计算叉积
    cross_product = v1(1) * v2(2) - v1(2) * v2(1);

    % 如果叉积为零，表示线段共线
    if abs(cross_product) < eps % 使用 eps 进行浮点数比较
        % 检查线段是否重叠
        if (min(p1(1), p2(1)) <= max(q1(1), q2(1)) && ...
            max(p1(1), p2(1)) >= min(q1(1), q2(1)) && ...
            min(p1(2), p2(2)) <= max(q1(2), q2(2)) && ...
            max(p1(2), p2(2)) >= min(q1(2), q2(2)))
            % 线段重叠，返回任意一个交点
            intersectionPoint = [min(max(p1(1), q1(1)), max(p1(2), q1(2))), ...
                                 min(max(p1(1), q1(1)), max(p1(2), q1(2)))];
        else
            % 线段平行且不重叠，没有交点
            intersectionPoint = [];
        end
    else
        % 计算交点的参数
        t1 = det([q1 - p1; v2]) / cross_product;
        t2 = det([p1 - q1; v1]) / (-cross_product);

        % 如果参数位于 [0, 1] 内，表示有交点
        if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1)
            intersectionPoint = p1 + t1 * v1;
        else
            % 没有交点
            intersectionPoint = [];
        end
    end
end
