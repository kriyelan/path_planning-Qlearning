function [path, tree] = rrt_star()
    % RRT*算法主函数
    % 输出:
    %   path - 从起点到终点的最优路径
    %   tree - 构建的RRT*树结构
    
    clc; clear; close all;
    
    %% ==================== 1. 参数设置 ====================
    % 环境参数
    map_size = [0, 20, 0, 20];  % 地图边界 [x_min, x_max, y_min, y_max]
    start_point = [2, 2];       % 起始点坐标
    goal_point = [18, 18];      % 目标点坐标
    goal_threshold = 1.0;       % 到达目标的距离阈值
    
    % 算法参数
    max_iterations = 2000;      % 最大迭代次数
    step_size = 1.0;           % 每次扩展的步长
    search_radius = 3.0;       % 重连搜索半径
    
    % 障碍物定义 (圆形障碍物: [x, y, radius])
    obstacles = [
        6, 6, 1.5;    % 障碍物1
        10, 8, 2.0;   % 障碍物2
        14, 12, 1.8;  % 障碍物3
        8, 15, 1.2;   % 障碍物4
        15, 5, 1.6    % 障碍物5
    ];
    
    %% ==================== 2. 初始化树结构 ====================
    % 树节点结构体包含: 位置、父节点索引、从起点到该点的代价
    tree.nodes = start_point;           % 节点位置 [x, y]
    tree.parent = 0;                   % 父节点索引(起始点没有父节点)
    tree.cost = 0;                     % 从起点到该节点的累积代价
    
    fprintf('开始RRT*算法搜索...\n');
    fprintf('起始点: (%.1f, %.1f)\n', start_point(1), start_point(2));
    fprintf('目标点: (%.1f, %.1f)\n', goal_point(1), goal_point(2));
    
    %% ==================== 3. 主循环 - RRT*算法核心 ====================
    for iter = 1:max_iterations
        % 3.1 随机采样
        if rand < 0.1  % 10%的概率采样目标点，增加算法效率
            rand_point = goal_point;
        else
            rand_point = random_sample(map_size);
        end
        
        % 3.2 寻找最近节点
        nearest_idx = find_nearest_node(tree, rand_point);
        nearest_node = tree.nodes(nearest_idx, :);
        
        % 3.3 向随机点方向扩展新节点
        new_node = steer(nearest_node, rand_point, step_size);
        
        % 3.4 碰撞检测
        if ~collision_free(nearest_node, new_node, obstacles)
            continue;  % 如果路径与障碍物碰撞，跳过这次迭代
        end
        
        % 3.5 在搜索半径内寻找所有邻近节点
        near_indices = find_near_nodes(tree, new_node, search_radius);
        
        % 3.6 选择最佳父节点（RRT*的关键改进）
        [best_parent_idx, min_cost] = choose_parent(tree, near_indices, new_node, obstacles);
        
        % 3.7 添加新节点到树中
        tree.nodes = [tree.nodes; new_node];
        tree.parent = [tree.parent; best_parent_idx];
        tree.cost = [tree.cost; min_cost];
        new_idx = size(tree.nodes, 1);
        
        % 3.8 重连操作（RRT*的另一个关键改进）
        tree = rewire(tree, near_indices, new_idx, obstacles);
        
        % 3.9 检查是否到达目标
        if norm(new_node - goal_point) <= goal_threshold
            fprintf('在第 %d 次迭代找到目标！\n', iter);
            break;
        end
        
        % 每500次迭代显示进度
        if mod(iter, 500) == 0
            fprintf('已完成 %d 次迭代，树中节点数: %d\n', iter, size(tree.nodes, 1));
        end
    end
    
    %% ==================== 4. 提取最优路径 ====================
    path = extract_path(tree, goal_point, goal_threshold);
    
    if isempty(path)
        fprintf('未找到有效路径！\n');
    else
        fprintf('找到路径！路径长度: %.2f\n', calculate_path_length(path));
    end
    
    %% ==================== 5. 可视化结果 ====================
    visualize_result(tree, obstacles, start_point, goal_point, path, map_size);
    
end

%% ==================== 辅助函数 ====================

function rand_point = random_sample(map_size)
    % 在地图范围内随机采样一个点
    % 输入: map_size - 地图边界
    % 输出: rand_point - 随机采样的点
    
    x = map_size(1) + rand * (map_size(2) - map_size(1));
    y = map_size(3) + rand * (map_size(4) - map_size(3));
    rand_point = [x, y];
end

function nearest_idx = find_nearest_node(tree, point)
    % 寻找树中距离给定点最近的节点
    % 输入: tree - 当前树结构, point - 目标点
    % 输出: nearest_idx - 最近节点的索引
    
    distances = sqrt(sum((tree.nodes - point).^2, 2));
    [~, nearest_idx] = min(distances);
end

function new_node = steer(from_node, to_node, step_size)
    % 从from_node向to_node方向以step_size步长扩展
    % 输入: from_node - 起始节点, to_node - 目标节点, step_size - 步长
    % 输出: new_node - 新扩展的节点
    
    direction = to_node - from_node;
    distance = norm(direction);
    
    if distance <= step_size
        new_node = to_node;
    else
        unit_direction = direction / distance;
        new_node = from_node + step_size * unit_direction;
    end
end

function is_free = collision_free(node1, node2, obstacles)
    % 检查两点间的直线路径是否与障碍物碰撞
    % 输入: node1, node2 - 路径两端点, obstacles - 障碍物列表
    % 输出: is_free - 布尔值，true表示无碰撞
    
    is_free = true;
    num_checks = 20;  % 路径上检查点的数量
    
    for i = 0:num_checks
        t = i / num_checks;
        check_point = (1 - t) * node1 + t * node2;
        
        % 检查是否与任何障碍物碰撞
        for j = 1:size(obstacles, 1)
            obs_center = obstacles(j, 1:2);
            obs_radius = obstacles(j, 3);
            
            if norm(check_point - obs_center) <= obs_radius + 0.1  % 0.1为安全边距
                is_free = false;
                return;
            end
        end
    end
end

function near_indices = find_near_nodes(tree, node, radius)
    % 找到距离给定节点在半径范围内的所有节点
    % 输入: tree - 树结构, node - 中心节点, radius - 搜索半径
    % 输出: near_indices - 邻近节点的索引列表
    
    distances = sqrt(sum((tree.nodes - node).^2, 2));
    near_indices = find(distances <= radius);
end

function [best_parent_idx, min_cost] = choose_parent(tree, near_indices, new_node, obstacles)
    % 为新节点选择最佳父节点（代价最小的可行连接）
    % 输入: tree - 树结构, near_indices - 候选父节点索引, new_node - 新节点, obstacles - 障碍物
    % 输出: best_parent_idx - 最佳父节点索引, min_cost - 最小代价
    
    min_cost = inf;
    best_parent_idx = near_indices(1);  % 默认选择第一个邻近节点
    
    for i = 1:length(near_indices)
        idx = near_indices(i);
        potential_parent = tree.nodes(idx, :);
        
        % 检查连接是否无碰撞
        if collision_free(potential_parent, new_node, obstacles)
            % 计算通过这个父节点到新节点的总代价
            cost = tree.cost(idx) + norm(new_node - potential_parent);
            
            if cost < min_cost
                min_cost = cost;
                best_parent_idx = idx;
            end
        end
    end
end

function tree = rewire(tree, near_indices, new_idx, obstacles)
    % 重连操作：检查是否可以通过新节点改善邻近节点的路径
    % 输入: tree - 树结构, near_indices - 邻近节点索引, new_idx - 新节点索引, obstacles - 障碍物
    % 输出: tree - 更新后的树结构
    
    new_node = tree.nodes(new_idx, :);
    new_cost = tree.cost(new_idx);
    
    for i = 1:length(near_indices)
        near_idx = near_indices(i);
        near_node = tree.nodes(near_idx, :);
        
        % 如果邻近节点是新节点的父节点，跳过
        if near_idx == tree.parent(new_idx)
            continue;
        end
        
        % 计算通过新节点到邻近节点的代价
        potential_cost = new_cost + norm(near_node - new_node);
        
        % 如果新路径代价更小且无碰撞，则重连
        if potential_cost < tree.cost(near_idx) && ...
           collision_free(new_node, near_node, obstacles)
            tree.parent(near_idx) = new_idx;
            tree.cost(near_idx) = potential_cost;
            
            % 更新所有后代节点的代价
            tree = update_descendants_cost(tree, near_idx);
        end
    end
end

function tree = update_descendants_cost(tree, node_idx)
    % 递归更新节点所有后代的代价
    % 输入: tree - 树结构, node_idx - 要更新的节点索引
    % 输出: tree - 更新后的树结构
    
    children = find(tree.parent == node_idx);
    
    for i = 1:length(children)
        child_idx = children(i);
        parent_node = tree.nodes(node_idx, :);
        child_node = tree.nodes(child_idx, :);
        
        % 更新子节点代价
        tree.cost(child_idx) = tree.cost(node_idx) + norm(child_node - parent_node);
        
        % 递归更新子节点的后代
        tree = update_descendants_cost(tree, child_idx);
    end
end

function path = extract_path(tree, goal_point, goal_threshold)
    % 从树中提取到目标点的最优路径
    % 输入: tree - 树结构, goal_point - 目标点, goal_threshold - 目标阈值
    % 输出: path - 路径点序列
    
    % 找到距离目标最近的节点
    distances = sqrt(sum((tree.nodes - goal_point).^2, 2));
    valid_indices = find(distances <= goal_threshold);
    
    if isempty(valid_indices)
        path = [];
        return;
    end
    
    % 在有效节点中选择代价最小的
    [~, best_idx] = min(tree.cost(valid_indices));
    goal_idx = valid_indices(best_idx);
    
    % 回溯构建路径
    path = [];
    current_idx = goal_idx;
    
    while current_idx ~= 0
        path = [tree.nodes(current_idx, :); path];
        current_idx = tree.parent(current_idx);
    end
end

function length = calculate_path_length(path)
    % 计算路径总长度
    % 输入: path - 路径点序列
    % 输出: length - 路径总长度
    
    length = 0;
    for i = 1:size(path, 1) - 1
        length = length + norm(path(i+1, :) - path(i, :));
    end
end

function visualize_result(tree, obstacles, start_point, goal_point, path, map_size)
    % 可视化RRT*搜索结果
    % 输入: tree - 树结构, obstacles - 障碍物, start_point - 起点, 
    %       goal_point - 终点, path - 路径, map_size - 地图边界
    
    figure('Position', [100, 100, 800, 600]);
    hold on;
    
    % 绘制地图边界
    rectangle('Position', [map_size(1), map_size(3), ...
              map_size(2)-map_size(1), map_size(4)-map_size(3)], ...
              'EdgeColor', 'k', 'LineWidth', 2);
    
    % 绘制障碍物
    for i = 1:size(obstacles, 1)
        center = obstacles(i, 1:2);
        radius = obstacles(i, 3);
        theta = linspace(0, 2*pi, 100);
        x_circle = center(1) + radius * cos(theta);
        y_circle = center(2) + radius * sin(theta);
        fill(x_circle, y_circle, [0.7, 0.7, 0.7], 'EdgeColor', 'k');
    end
    
    % 绘制RRT*树
    for i = 2:size(tree.nodes, 1)  % 从第2个节点开始（跳过起点）
        parent_idx = tree.parent(i);
        child_node = tree.nodes(i, :);
        parent_node = tree.nodes(parent_idx, :);
        
        plot([parent_node(1), child_node(1)], [parent_node(2), child_node(2)], ...
             'c-', 'LineWidth', 0.5);
    end
    
    % 绘制所有节点
    plot(tree.nodes(:, 1), tree.nodes(:, 2), 'c.', 'MarkerSize', 3);
    
    % 绘制起点和终点
    plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(goal_point(1), goal_point(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    % 绘制最优路径
    if ~isempty(path)
        plot(path(:, 1), path(:, 2), 'b-', 'LineWidth', 3);
        plot(path(:, 1), path(:, 2), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    end
    
    % 设置图形属性
    axis equal;
    xlim([map_size(1)-1, map_size(2)+1]);
    ylim([map_size(3)-1, map_size(4)+1]);
    grid on;
    
    title('RRT* 路径规划算法结果', 'FontSize', 14, 'FontWeight', 'bold');
    legend('地图边界', '障碍物', 'RRT*树', '树节点', '起点', '终点', '最优路径', ...
           'Location', 'best');
    xlabel('X坐标');
    ylabel('Y坐标');
    
    fprintf('可视化完成！\n');
    fprintf('绿色圆点: 起始点\n');
    fprintf('红色圆点: 目标点\n');
    fprintf('青色线段: RRT*搜索树\n');
    fprintf('蓝色粗线: 找到的最优路径\n');
end