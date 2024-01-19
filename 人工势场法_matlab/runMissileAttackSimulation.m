function runMissileAttackSimulation()
  % 船的初始位置和速度
shipPosition = 0;
shipSpeed = 5;
 
% 导弹的初始位置和速度（相对于船的初始位置）
missilePosition = 0;  % 修正导弹初始位置
missileSpeed = 20;
 
% 模拟时间参数
totalTime = 10;
timeStep = 0.1;

% 初始化轨迹数组
shipTrajectory = [];
missileTrajectory = [];
targetTrajectory = [];
 
% 记录导弹发射的位置
missileLaunchPosition = [];
 
% 获取目标打击点坐标
targetPosition = getTargetPosition();

%作弊是吧
missileSpeed = (targetPosition(2)-shipPosition)/totalTime-shipSpeed;

% 模拟导弹攻击过程
for t = 0:timeStep:totalTime
    % 船移动
    shipPosition = shipPosition + shipSpeed * timeStep;
    shipTrajectory = [shipTrajectory, shipPosition];
 
    % 目标保持静止
    targetTrajectory = [targetTrajectory, targetPosition(2)];  % 只关心Y轴的坐标
 
    % 导弹更新轨迹（相对于船的初始位置）
    missilePosition = missileSpeed * t;  % 修正导弹更新公式
    missileTrajectory = [missileTrajectory, shipPosition + missilePosition];%速度不变的话倒也合理
 
    % 记录导弹发射的位置
    if t == 0
        missileLaunchPosition = shipPosition;
    end
 
    % 判定是否击中目标
    if abs(missilePosition - targetPosition(2)) < 1e-6
        disp('目标击中！');
        break;
    end
end

% 绘制船、导弹和目标的轨迹
figure;
timesteps = linspace(0, totalTime, length(shipTrajectory));
plot(timesteps, shipTrajectory, 'LineWidth', 2, 'DisplayName', '船');
hold on;
plot(timesteps, missileTrajectory, 'LineWidth', 2, 'DisplayName', '导弹');
plot(timesteps, targetTrajectory, 'LineWidth', 2, 'DisplayName', '目标');
scatter(0, missileLaunchPosition, 100, 'filled', 'MarkerFaceColor', 'g', 'DisplayName', '导弹发射位置');
scatter(totalTime, targetPosition(2), 100, 'filled', 'MarkerFaceColor', 'r', 'DisplayName', '目标位置');
xlabel('时间 (秒)');
ylabel('位置');
title('船舶发射导弹攻击目标');
legend;
grid on;
hold off;
end
 
function targetPosition = getTargetPosition()
    prompt = {'输入目标打击点的X坐标:', '输入目标打击点的Y坐标:'};
    dlgtitle = '输入目标打击点坐标';
    dims = [1 35];
    definput = {'', ''};
    answer = inputdlg(prompt, dlgtitle, dims, definput);
 
    % 转换为数字
    x = str2double(answer{1});
    y = str2double(answer{2});
 
    % 返回目标打击点坐标
    targetPosition = [x, y];
end
