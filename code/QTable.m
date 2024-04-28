classdef QTable < handle
    properties
        gamma % 折扣因子
        alpha % 更新步长
        epsilon % 贪婪算法的贪婪系数
        Table % Q值表
    end

    methods
        function obj = QTable(G, A, E)
            obj.gamma= G; % 折扣因子
            obj.alpha= A; % 学习率
            obj.epsilon= E; % 贪婪算法的贪婪系数
        end
        function Update_alpha(obj, A)
            obj.alpha= A;
        end
        function Update_gamma(obj, G)
            obj.gamma= G;
        end
        function Update_epsilon(obj, E)
            obj.gamma= E;
        end
        function initialization(obj, Q_ini)
            for i = 1:1024
                dataindex = i - 1;
                dataString = dec2bin(dataindex);
                while length(dataString) < 10
                    dataString = strcat('0', dataString);
                end
                obj.Table(i).data = dataString;
                obj.Table(i).action = [0, -1, 1, -2, 2];
                obj.Table(i).q = [Q_ini, Q_ini, Q_ini, Q_ini, Q_ini];
            end
        end

        function [action_index, action] = SelectAction_train(obj, turtlebot)
            if rand < obj.epsilon
                action_index = randi(5);
            else
                [~, action_index] = max(obj.Table(turtlebot.ProcessedData_2).q);
            end
            action = obj.Table(turtlebot.ProcessedData_2).action(action_index);
        end

        function [action_index, action] = SelectAction_test(obj, turtlebot)
            [~, action_index] = max(obj.Table(turtlebot.ProcessedData_2).q);
            action = obj.Table(turtlebot.ProcessedData_2).action(action_index);
        end

        function UpdateNet(obj, data, action_index, reward)
            obj.Table(data).q(action_index) = (1 - obj.alpha) * obj.Table(data).q(action_index) + obj.alpha * reward + obj.gamma * max(obj.Table(data).q);
%             if (min(obj.Table(data).q) ~= 0)
%                 obj.Table(data).q = 10 * obj.Table(data).q/-min(obj.Table(data).q);
%             end
        end
    end
end

