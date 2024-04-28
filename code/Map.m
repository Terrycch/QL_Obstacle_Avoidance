classdef Map < handle
    properties
        MapSize     % 地图尺寸 [width, height]
        Center      % 地图中心点坐标 [x, y]
        Obstacle_list
        MaxDiatance % 地图最大距离
        Trajectory  % 轨迹
    end

    methods
        % 构造函数
        function obj = Map(mapSize, center)
            obj.MapSize = mapSize;
            obj.Center = center;
            obj.MaxDiatance = sqrt(obj.MapSize(1)^2 + obj.MapSize(2)^2);
            obj.Obstacle_list = [];
        end
        %
        function map_initial(obj)
            obj.Obstacle_list = [];
        end
        function add_obs(obj,obs)
            obj.Obstacle_list = [obj.Obstacle_list, obs];
        end
        %绘制
        function add_trajectory_obs(obj,number)
            for i = 1:number
                if i ~= (number+1)/2
                    index = floor(i * length(obj.Trajectory)/(number + 1));

                    switch randi(4)
                        case 1
                            offset = -0.2;
                        case 2
                            offset = -0.1;
                        case 3
                            offset = 0.1;
                        case 4
                            offset = 0.2;
                    end
%                     offset = -0.2 + 0.4 *rand; %横向偏移量
                    location = [obj.Trajectory(index).x + offset * sin(obj.Trajectory(index).theta), obj.Trajectory(index).y - offset * cos(obj.Trajectory(index).theta)];
                    size = 0.1;
                    obs = Obstacle(0, location, size, 1);
                    obj.Obstacle_list = [obj.Obstacle_list,  obs];
                end
            end
        end
        %
        function change_trajectory(obj, trajectory)
            obj.Trajectory = trajectory;
        end
        function plotMap(obj)
            % 创建地图图形
            hold on;
            axis equal;

            % 绘制地图边界
            mapSize = obj.MapSize;
            mapCenter = obj.Center;
            mapRect = [-mapSize(1)/2+mapCenter(1), mapSize(1)/2+mapCenter(1), -mapSize(2)/2+mapCenter(2), mapSize(2)/2+mapCenter(2)];
            rectangle('Position', [mapRect(1), mapRect(3), mapSize(1), mapSize(2)], 'EdgeColor', 'k', 'LineWidth', 2);

            % 遍历障碍物并绘制
            for i = 1:length(obj.Obstacle_list)
                obstacle = obj.Obstacle_list(i);
                if (obstacle.Visible == 1)
                    if obstacle.Type == 0
                        % 绘制圆形障碍物
                        center = obstacle.Position;
                        radius = obstacle.Size;
                        viscircles(center, radius, 'EdgeColor', 'k');
                    elseif obstacle.Type == 1
                        % 绘制矩形障碍物
                        x = obstacle.Position;
                        width = 2*obstacle.Size;
                        height = 2*obstacle.Size;
                        rectangle('Position', [x(1)-width/2, x(2)-height/2, width, height], 'EdgeColor', 'k', 'FaceColor', 'k');
                    end
                end
            end
            
            xData = [obj.Trajectory.x];
            yData = [obj.Trajectory.y];
            pm = plot(xData, yData, 'r:', LineWidth=1.5);

            % 设置坐标轴范围
            xlim([mapRect(1), mapRect(2)]);
            ylim([mapRect(3), mapRect(4)]);
            % 添加标题
            drawnow;
        end
    end
end