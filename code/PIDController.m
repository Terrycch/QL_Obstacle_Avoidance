classdef PIDController < handle
    properties
        Kp
        Ki
        Kd
        target
        prevError
        integral
        Limit
    end
    
    methods
        % 构造函数，用于初始化参数
        function obj = PIDController(Kp, Ki, Kd, limit)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.target = 0;
            obj.prevError = 0;
            obj.integral = 0;
            obj.Limit = limit;
        end
        
        % 更新控制器状态并计算输出
        function output = update(obj, current)
            error = obj.target - current;
            obj.integral = obj.integral + error;
            derivative = error - obj.prevError;
            
            output = obj.Kp * error + obj.Ki * obj.integral + obj.Kd * derivative;
            output = output_limit(output, obj.Limit);
            obj.prevError = error;
        end
        
        % 更改目标值
        function setTarget(obj, newTarget)
            obj.target = newTarget;
        end
        
        % 重置控制器状态
        function reset(obj)
            obj.prevError = 0;
            obj.integral = 0;
        end
    end
end

function output = output_limit(output, limit)
    if output > limit(2)
        output = limit(2);
    elseif output < limit(1)
        output = limit(1);
    end
end
