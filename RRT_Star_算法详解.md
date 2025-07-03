# RRT*算法MATLAB实现详细解释

## 概述

RRT*(Rapidly-exploring Random Tree Star)是一种基于采样的路径规划算法，它是传统RRT算法的改进版本。RRT*算法具有渐近最优性，能够随着迭代次数增加逐渐收敛到最优路径。

## 算法核心思想

与传统RRT算法相比，RRT*增加了两个关键改进：
1. **Choose Parent（选择父节点）**：为新节点选择代价最小的父节点
2. **Rewire（重连）**：通过新节点改善邻近节点的路径

## 代码详细解释

### 1. 主函数结构

```matlab
function [path, tree] = rrt_star()
```

**功能**: 主函数入口，返回找到的最优路径和构建的树结构。

**关键特点**:
- 使用结构化编程，将算法分为参数设置、初始化、主循环、路径提取和可视化五个部分
- 返回完整的树结构，便于分析算法性能

### 2. 参数设置部分

```matlab
%% ==================== 1. 参数设置 ====================
map_size = [0, 20, 0, 20];  % 地图边界
start_point = [2, 2];       % 起始点
goal_point = [18, 18];      % 目标点
goal_threshold = 1.0;       % 到达目标的距离阈值
max_iterations = 2000;      % 最大迭代次数
step_size = 1.0;           % 每次扩展的步长
search_radius = 3.0;       % 重连搜索半径
```

**参数意义**:
- `map_size`: 定义工作空间边界，格式为[x_min, x_max, y_min, y_max]
- `goal_threshold`: 当节点与目标距离小于此值时认为到达目标
- `step_size`: 控制树的扩展步长，影响算法的探索精度
- `search_radius`: RRT*特有参数，决定重连操作的搜索范围

**障碍物定义**:
```matlab
obstacles = [
    6, 6, 1.5;    % [x, y, radius] 格式
    10, 8, 2.0;
    14, 12, 1.8;
    8, 15, 1.2;
    15, 5, 1.6
];
```

### 3. 树结构初始化

```matlab
tree.nodes = start_point;   % 节点位置矩阵
tree.parent = 0;           % 父节点索引数组
tree.cost = 0;             % 代价数组
```

**数据结构设计**:
- `tree.nodes`: N×2矩阵，存储所有节点的(x,y)坐标
- `tree.parent`: N×1向量，存储每个节点的父节点索引
- `tree.cost`: N×1向量，存储从起点到每个节点的累积代价

### 4. 主循环 - RRT*算法核心

#### 4.1 随机采样

```matlab
if rand < 0.1  % 10%的概率采样目标点
    rand_point = goal_point;
else
    rand_point = random_sample(map_size);
end
```

**采样策略**:
- **目标偏向采样**: 10%概率直接采样目标点，提高算法效率
- **均匀随机采样**: 90%概率在整个空间均匀采样，保证探索性

#### 4.2 寻找最近节点

```matlab
function nearest_idx = find_nearest_node(tree, point)
    distances = sqrt(sum((tree.nodes - point).^2, 2));
    [~, nearest_idx] = min(distances);
end
```

**算法分析**:
- 计算所有节点到随机点的欧几里德距离
- 时间复杂度: O(n)，其中n是当前树中节点数
- 可优化为k-d树结构以提高效率

#### 4.3 节点扩展 (Steer函数)

```matlab
function new_node = steer(from_node, to_node, step_size)
    direction = to_node - from_node;
    distance = norm(direction);
    
    if distance <= step_size
        new_node = to_node;
    else
        unit_direction = direction / distance;
        new_node = from_node + step_size * unit_direction;
    end
end
```

**功能解析**:
- 限制每次扩展的最大距离为`step_size`
- 如果目标点在步长范围内，直接连接
- 否则沿方向向量扩展固定步长
- 保证树的增长是渐进且可控的

#### 4.4 碰撞检测

```matlab
function is_free = collision_free(node1, node2, obstacles)
    is_free = true;
    num_checks = 20;  % 路径离散化检查点数量
    
    for i = 0:num_checks
        t = i / num_checks;
        check_point = (1 - t) * node1 + t * node2;
        
        for j = 1:size(obstacles, 1)
            obs_center = obstacles(j, 1:2);
            obs_radius = obstacles(j, 3);
            
            if norm(check_point - obs_center) <= obs_radius + 0.1
                is_free = false;
                return;
            end
        end
    end
end
```

**碰撞检测策略**:
- **路径离散化**: 将连续路径分成20个离散点检查
- **安全边距**: 增加0.1的安全距离避免擦边碰撞
- **早停机制**: 一旦检测到碰撞立即返回false

#### 4.5 RRT*核心改进1: 选择最佳父节点

```matlab
function [best_parent_idx, min_cost] = choose_parent(tree, near_indices, new_node, obstacles)
    min_cost = inf;
    best_parent_idx = near_indices(1);
    
    for i = 1:length(near_indices)
        idx = near_indices(i);
        potential_parent = tree.nodes(idx, :);
        
        if collision_free(potential_parent, new_node, obstacles)
            cost = tree.cost(idx) + norm(new_node - potential_parent);
            
            if cost < min_cost
                min_cost = cost;
                best_parent_idx = idx;
            end
        end
    end
end
```

**算法意义**:
- **代价最优化**: 在所有可行连接中选择代价最小的父节点
- **渐近最优性保证**: 这是RRT*相比RRT的关键改进
- **局部搜索**: 只在搜索半径内的节点中选择

#### 4.6 RRT*核心改进2: 重连操作

```matlab
function tree = rewire(tree, near_indices, new_idx, obstacles)
    new_node = tree.nodes(new_idx, :);
    new_cost = tree.cost(new_idx);
    
    for i = 1:length(near_indices)
        near_idx = near_indices(i);
        near_node = tree.nodes(near_idx, :);
        
        if near_idx == tree.parent(new_idx)
            continue;  % 跳过新节点的父节点
        end
        
        potential_cost = new_cost + norm(near_node - new_node);
        
        if potential_cost < tree.cost(near_idx) && ...
           collision_free(new_node, near_node, obstacles)
            tree.parent(near_idx) = new_idx;
            tree.cost(near_idx) = potential_cost;
            tree = update_descendants_cost(tree, near_idx);
        end
    end
end
```

**重连机制解析**:
- **路径优化**: 检查是否可以通过新节点改善邻近节点的路径
- **代价传播**: 更新被重连节点及其所有后代的代价
- **渐近最优性**: 随着迭代增加，路径质量不断改善

#### 4.7 后代代价更新

```matlab
function tree = update_descendants_cost(tree, node_idx)
    children = find(tree.parent == node_idx);
    
    for i = 1:length(children)
        child_idx = children(i);
        parent_node = tree.nodes(node_idx, :);
        child_node = tree.nodes(child_idx, :);
        
        tree.cost(child_idx) = tree.cost(node_idx) + norm(child_node - parent_node);
        tree = update_descendants_cost(tree, child_idx);  % 递归更新
    end
end
```

**递归更新机制**:
- **深度优先遍历**: 递归更新所有后代节点的代价
- **代价一致性**: 保证树中所有节点的代价信息正确
- **性能考虑**: 递归深度受树的结构影响

### 5. 路径提取

```matlab
function path = extract_path(tree, goal_point, goal_threshold)
    distances = sqrt(sum((tree.nodes - goal_point).^2, 2));
    valid_indices = find(distances <= goal_threshold);
    
    if isempty(valid_indices)
        path = [];
        return;
    end
    
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
```

**路径提取策略**:
- **目标区域**: 找到所有在目标阈值内的节点
- **最优选择**: 在可行节点中选择代价最小的
- **路径回溯**: 沿父节点链表回溯到起点

### 6. 可视化系统

```matlab
function visualize_result(tree, obstacles, start_point, goal_point, path, map_size)
    % 绘制地图边界
    % 绘制障碍物（圆形）
    % 绘制RRT*搜索树
    % 绘制最优路径
    % 添加图例和标签
end
```

**可视化要素**:
- **搜索树**: 青色线段显示算法的探索过程
- **最优路径**: 蓝色粗线突出显示找到的解
- **障碍物**: 灰色填充圆形
- **起终点**: 绿色和红色圆点标记

## 算法性能分析

### 时间复杂度
- **单次迭代**: O(n)，其中n是当前树节点数
- **整体算法**: O(n²)，考虑到迭代过程中树的增长

### 空间复杂度
- **树存储**: O(n)，存储节点、父节点和代价信息
- **辅助空间**: O(k)，其中k是搜索半径内的节点数

### 收敛性质
- **概率完备性**: 如果解存在，算法必定能找到
- **渐近最优性**: 随着样本数增加，解收敛到最优路径

## 参数调优建议

### 步长 (step_size)
- **较小值**: 精确度高，但收敛慢
- **较大值**: 收敛快，但可能错过窄通道

### 搜索半径 (search_radius)
- **较小值**: 计算效率高，但优化能力有限
- **较大值**: 优化能力强，但计算开销大

### 推荐设置
```matlab
step_size = min(map_width, map_height) / 20;        % 地图大小的5%
search_radius = 2 * step_size;                      % 步长的2倍
max_iterations = 1000 * log(map_area);              % 基于地图面积
```

## 算法扩展

### 1. 双向RRT*
同时从起点和终点构建树，提高搜索效率

### 2. Informed RRT*
利用启发式信息指导采样，专注于有希望的区域

### 3. RRT*-Smart
动态调整参数，在探索和利用之间取得平衡

## 使用方法

1. **运行算法**:
```matlab
[path, tree] = rrt_star();
```

2. **自定义参数**:
修改函数内的参数设置部分

3. **分析结果**:
- `path`: 最优路径点序列
- `tree`: 完整的搜索树结构

## 常见问题解决

### 1. 算法收敛慢
- 增加目标偏向采样概率
- 适当增大步长
- 减小搜索半径

### 2. 内存不足
- 减少最大迭代次数
- 实现节点修剪机制
- 使用更高效的数据结构

### 3. 路径质量差
- 增加迭代次数
- 适当增大搜索半径
- 调整目标阈值

这个RRT*实现提供了一个完整且高效的路径规划解决方案，适用于二维环境下的机器人路径规划、无人机航迹规划等应用场景。