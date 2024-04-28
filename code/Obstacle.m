classdef Obstacle
    properties
        Type      % 障碍物类型 0(cir) or 1(rec)
        Position  % 障碍物位置信息，矩形为中心点坐标，圆形为圆心坐标
        Size      % 障碍物大小信息，矩形为宽度和高度，圆形为半径
        Visible   % 是否可视
    end
    
    methods
        % 构造函数
        function obj = Obstacle(type, position, size, Visible)
            obj.Type = type;
            obj.Position = position;
            obj.Size = size;
            obj.Visible = Visible;
        end
    end
end