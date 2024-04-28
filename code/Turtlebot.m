classdef Turtlebot < handle
    properties
        Position    % 机器人位置 [x, y, theta]
        %
        SensorSize % 雷达数据数量
        SensorLength % 雷达测距最大长度
        BodySize % 机器人大小（半径）
        ObsAvoDistance % 避障半径
        MaxVel % 最大速度
        %
        Action     % 机器人速度 [v, w]
        net_action % 避障动作
        SensorData % 雷达数据
        GoalData % 目标数据
        ProcessedData % 观测数据(雷达数据均值[SensorSize])
        ProcessedData_2 % 观测数据二进制处理
        CollisionStatus % 碰撞状态（0：正常，1：碰撞）
        %
        CacheData % 缓存数据，上一时刻(x,y,theta,观测数据[SensorSize], 速度,角速度)
    end
    
    methods
        % 构造函数
        function obj = Turtlebot(location)
            obj.Position.x = location(1);
            obj.Position.y = location(2);
            obj.Position.angle = location(3);
            %%%
            obj.SensorSize = 10;
            obj.SensorLength = 1;
            obj.BodySize = 0.1;
            obj.ObsAvoDistance = 0.5;
            obj.MaxVel.v = 0.22;
            obj.MaxVel.w = 2.84;
            %%%
            obj.Action.v = 0;
            obj.Action.w = 0;
            obj.net_action = 0;
            %%%
            obj.SensorData = obj.SensorLength*ones(1, obj.SensorSize);
            obj.GoalData.l = 0;
            obj.GoalData.theta = 0;
            %%%
            obj.ProcessedData = zeros(1, obj.SensorSize);
            obj.ProcessedData_2 = 0;
            %%%
            obj.CollisionStatus = 0;
            %%%
            obj.CacheData.position = obj.Position;
            obj.CacheData.action = obj.Action;
            obj.CacheData.data = obj.SensorData;
            obj.CacheData.l = obj.GoalData.l;
            obj.CacheData.theta = obj.GoalData.theta;
            obj.CacheData.iscollision = 0;
            %%%
        end
        function initialization(obj, location)
            obj.Position.x = location(1);
            obj.Position.y = location(2);
            obj.Position.angle = location(3);
            %%%
            obj.Action.v = 0;
            obj.Action.w = 0;
            obj.net_action = 0;
            %%%
            obj.SensorData = obj.SensorLength*ones(1, obj.SensorSize);
            obj.GoalData.l = 0;
            obj.GoalData.theta = 0;
            %%%
            obj.ProcessedData = zeros(1, obj.SensorSize);
            obj.ProcessedData_2 = 0;
            %%%
            obj.CollisionStatus = 0;
            %%%
            obj.CacheData.position = obj.Position;
            obj.CacheData.action = obj.Action;
            obj.CacheData.data = obj.SensorData;
            obj.CacheData.l = obj.GoalData.l;
            obj.CacheData.theta = obj.GoalData.theta;
            obj.CacheData.iscollision = 0;
            %%%
        end
        % 更新机器人位置
        function updatePosition(obj, sampletime)
            % 更新偏航角

            obj.CacheData.position = obj.Position;%存入缓存
            obj.Position.angle = obj.Position.angle + obj.Action.w * sampletime;

            if(obj.Position.angle > pi)
                obj.Position.angle = obj.Position.angle - 2*pi;
            elseif(obj.Position.angle < -pi)
                obj.Position.angle = obj.Position.angle + 2*pi;
            end
            % 计算机器人的位移
            displacement = obj.Action.v * sampletime;
            % 更新机器人的位置
            obj.Position.x = obj.Position.x + displacement * cos(obj.Position.angle);
            obj.Position.y = obj.Position.y + displacement * sin(obj.Position.angle);
        end
        % 显示机器人位置
        function plotturtlebot(obj)
            % 绘制机器人
            rectangle('Position',[obj.Position.x - obj.BodySize, obj.Position.y - obj.BodySize, 2*obj.BodySize, 2*obj.BodySize],'Curvature',[1,1])
            drawnow;
            % 绘制箭头
%             quiver(obj.Position.x, obj.Position.y,...
%                 obj.LinearVelocity*cos(obj.Position.angle), obj.LinearVelocity*sin(obj.Position.angle), 'Color', 'b', 'LineWidth', 1);
        end
        % 计算目标点参数
        function goal_processing(obj, goal_point)
            obj.CacheData.l = obj.GoalData.l;
            obj.CacheData.theta = obj.GoalData.theta;
            obj.GoalData.x = goal_point(1);
            obj.GoalData.y = goal_point(2);
            obj.GoalData.l = sqrt((obj.GoalData.x-obj.Position.x)^2 + (obj.GoalData.y-obj.Position.y)^2);
            obj.GoalData.theta = atan2((obj.GoalData.y-obj.Position.y), (obj.GoalData.x-obj.Position.x)) - obj.Position.angle;
            if(obj.GoalData.theta > pi)
                obj.GoalData.theta = obj.GoalData.theta - 2*pi;
            elseif(obj.GoalData.theta < -pi)
                obj.GoalData.theta = obj.GoalData.theta + 2*pi;
            end
        end
        % 模拟传感器测量
        function measureSensors(obj, map)
            obj.CacheData.data = obj.SensorData;
            % 计算每个传感器的角度
            sensorAngles = linspace(-pi/2, pi/2, obj.SensorSize);
            
            for i = 1:obj.SensorSize
                distance = obj.SensorLength;
                currentAngle = sensorAngles(i) + obj.Position.angle;
                % 计算传感器末端坐标
                sensorX = obj.Position.x + obj.SensorLength * cos(currentAngle);
                sensorY = obj.Position.y + obj.SensorLength * sin(currentAngle);
                sensor = [sensorX, sensorY];
                for j = 1:length(map.Obstacle_list)
                    if ( (sqrt((map.Obstacle_list(j).Position(1)-obj.Position.x)^2 ...
                            + (map.Obstacle_list(j).Position(2)-obj.Position.y)^2 ) ...
                            - map.Obstacle_list(j).Size)<=obj.SensorLength)
                        borderline = findTangentAndIntersection([obj.Position.x, obj.Position.y],map.Obstacle_list(j).Position, map.Obstacle_list(j).Size);
                        intersection_point = findLineSegmentIntersection([obj.Position.x, obj.Position.y],sensor,borderline(1,:),borderline(2,:));
                        if (~isempty(intersection_point))
                            current_diatance = sqrt((intersection_point(1)-obj.Position.x)^2+(intersection_point(2)-obj.Position.y)^2);
                            if (current_diatance < distance)
                                distance = current_diatance;
                            end
                        end
                    end
                end
%                 for x = 1:2
%                     for y = 1:2
%                         intersection_point = findLineSegmentIntersection([obj.Position.x, obj.Position.y],sensor,...
%                             [map.Center(1)+((-1)^x)*map.MapSize(1)/2,map.Center(2)+((-1)^x)*map.MapSize(2)/2],...
%                             [map.Center(1)+((-1)^y)*map.MapSize(1)/2,map.Center(2)-((-1)^y)*map.MapSize(2)/2]);
%                         if (~isempty(intersection_point))
%                             current_diatance = sqrt((intersection_point(1)-obj.Position.x)^2+(intersection_point(2)-obj.Position.y)^2);
%                             if (current_diatance < distance)
%                                 distance = current_diatance;
%                             end
%                         end
%                     end
%                 end
                obj.SensorData(i) = (distance);
            end
        end
        % 模拟传感器显示
        function showSensor(obj)
            angles = linspace(0, 180, obj.SensorSize);
            polarplot(deg2rad(angles),obj.SensorData.data, '-o');
            drawnow;
            title('障碍物雷达');
        end
        % 处理数据
        function Data_processing(obj)
            obj.CacheData.data = obj.ProcessedData;%存入缓存
            %观测数据
            for i = 1:obj.SensorSize
                if obj.SensorData(i) < obj.ObsAvoDistance
                    obj.ProcessedData(i) = 1;
                else
                    obj.ProcessedData(i) = 0;
                end
            end
            binStr = num2str(obj.ProcessedData);

            % 将二进制字符串转换为十进制数并加一
            decNum = bin2dec(binStr) + 1;

            % 输出结果
            obj.ProcessedData_2 = decNum;
        end
        % 检测碰撞
        function isCollision(obj, map)
            obj.CacheData.iscollision = obj.CollisionStatus;
            for j = 1:length(map.Obstacle_list)
                if((sqrt((map.Obstacle_list(j).Position(1)-obj.Position.x)^2 ...
                        + (map.Obstacle_list(j).Position(2)-obj.Position.y)^2)...
                        - map.Obstacle_list(j).Size - obj.BodySize) <= 0)
                    obj.CollisionStatus = 1;
                    break;
                end
            end
%             if (obj.CollisionStatus == 0)
%                 if (((obj.Position.x-obj.BodySize) <= (map.Center(1)-map.MapSize(1)/2)) || ...
%                         ((obj.Position.x+obj.BodySize) >= (map.Center(1)+map.MapSize(1)/2)) || ...
%                         ((obj.Position.y-obj.BodySize) <= (map.Center(2)-map.MapSize(2)/2)) || ...
%                         ((obj.Position.y+obj.BodySize) >= (map.Center(2)+map.MapSize(2)/2)))
%                     obj.CollisionStatus = 1;
%                 end
%             end
        end
         function reward = getreward(obj)
             reward = 0;
             if (obj.CollisionStatus == 1 && obj.CacheData.iscollision == 0)
                reward = - 100;
%              else
%                  reward = - 0.001* (abs(obj.CacheData.action - obj.net_action) + abs(obj.net_action));
             end
        end
    end
end
