%%
clc;clear;close all;
%%
total_time = 100;
sample_time = 0.1;%采样时间
%
map_test = Map([5, 5], [2.5, 2.5]);
map_test.change_trajectory(generateTrajectory(sample_time, "circle"));

map_test.add_obs(Obstacle(0, [0.9, 0.9], 0.2, 1));

map_test.add_obs(Obstacle(0, [1.5, 3.8], 0.3, 1));
map_test.add_obs(Obstacle(0, [3.5, 1.2], 0.3, 1));

map_test.add_obs(Obstacle(0, [4.1, 4.1], 0.2, 1));

turtlebot = Turtlebot([map_test.Trajectory(1).x-0.2 , map_test.Trajectory(1).y, map_test.Trajectory(1).theta]);

%%
w_area = [-2.84, 2.84];
w_pid = PIDController(3, 0.5, 1, w_area);
v_area = [-0.22, 0.22];
v_pid = PIDController(2.5, 0.1, 0.1, v_area);
load('Q_table_final.mat');

%%
%%
w_pid.reset();
v_pid.reset();

turtlebot.initialization([map_test.Trajectory(1).x-0.2 , map_test.Trajectory(1).y, map_test.Trajectory(1).theta]);
w_pid.reset();
v_pid.reset();
trajectory_3 = [turtlebot.Position.x,turtlebot.Position.y];
w_data_3 = [];
v_data_3 = [];
t = 0;

for index = 1:length(map_test.Trajectory)
    timeleft_3 = cputime;
    turtlebot.measureSensors(map_test);
    turtlebot.Data_processing();
    turtlebot.goal_processing([map_test.Trajectory(index).x, map_test.Trajectory(index).y]);
    turtlebot.CacheData.action = turtlebot.net_action;
    [action_index, turtlebot.net_action] = Q_Table.SelectAction_test(turtlebot);
    w_pid.setTarget(atan((turtlebot.CacheData.action + turtlebot.net_action)/2));
    v_pid.setTarget(-0.2);
    turtlebot.Action.w = w_pid.update(-turtlebot.GoalData.theta);
    turtlebot.Action.v = v_pid.update(-turtlebot.GoalData.l);
    turtlebot.updatePosition(sample_time);
    turtlebot.isCollision(map_test);
    t = t + sample_time;
    w_data_3= [w_data_3;t, turtlebot.Action.w];
    v_data_3= [v_data_3;t, turtlebot.Action.v];
    trajectory_3 = [trajectory_3;turtlebot.Position.x,turtlebot.Position.y];
    if turtlebot.CollisionStatus == 1
        break;
    end
end

%%
figure(1);
subplot(2, 2, [1,3])
map_test.plotMap();
hold on;
turtlebot.plotturtlebot();
hold on;
p13 = plot(trajectory_3(:,1),trajectory_3(:,2), 'Color', "#0072BD", LineWidth=1.5);
hold off;
%%
w_ref = [map_test.Trajectory.w];
v_ref = [map_test.Trajectory.v];
w_lim_1 = w_area(1) * ones(length(w_ref));
w_lim_2 = w_area(2) * ones(length(w_ref));
v_lim = v_area(2) * ones(length(v_ref));

subplot(2, 2, 2)
hold on;
p23 = plot(w_data_3(:,1),w_data_3(:,2), 'Color', "#0072BD", LineWidth=0.5);
p25 = plot(w_data_3(:,1),w_ref, 'Color', "#A2142F", LineWidth=1);
xlim([0,total_time]);
ylim([-4, 3.5]);
hold on;
drawnow;
hold on;

subplot(2, 2, 4)
hold on;
xlabel('Time(s)');ylabel('Linear velocities');
title('(c)')
p33 = plot(v_data_3(:,1),v_data_3(:, 2), 'Color', "#0072BD", LineWidth=0.5);
p34 = plot(v_data_3(:,1),v_ref, 'Color', "#A2142F", LineWidth=1);
p35 = plot(v_data_3(:,1),v_lim, 'r--', LineWidth=0.5);
xlim([0,total_time]);
ylim([0,0.25]);
hold on;