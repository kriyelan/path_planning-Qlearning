% RRT*算法测试脚本
% 这个脚本演示如何使用RRT*算法进行路径规划

clc; clear; close all;

fprintf('====== RRT*算法测试脚本 ======\n\n');

%% 1. 基本测试 - 使用默认参数
fprintf('1. 运行基本RRT*算法测试...\n');
[path, tree] = rrt_star();

if ~isempty(path)
    fprintf('✓ 成功找到路径！\n');
    fprintf('  路径包含 %d 个节点\n', size(path, 1));
    fprintf('  路径总长度: %.2f\n', calculate_path_length_test(path));
    fprintf('  搜索树包含 %d 个节点\n', size(tree.nodes, 1));
else
    fprintf('✗ 未找到路径\n');
end

%% 等待用户查看结果
fprintf('\n按任意键继续下一个测试...\n');
pause;

%% 2. 复杂环境测试 - 修改参数
fprintf('\n2. 运行复杂环境测试...\n');
run_complex_environment_test();

%% 等待用户查看结果
fprintf('\n按任意键继续性能分析...\n');
pause;

%% 3. 性能分析测试
fprintf('\n3. 运行性能分析测试...\n');
run_performance_analysis();

fprintf('\n====== 测试完成 ======\n');

%% 辅助函数

function length = calculate_path_length_test(path)
    % 计算路径总长度（测试用）
    length = 0;
    if size(path, 1) < 2
        return;
    end
    
    for i = 1:size(path, 1) - 1
        length = length + norm(path(i+1, :) - path(i, :));
    end
end

function run_complex_environment_test()
    % 复杂环境测试
    fprintf('设置复杂环境参数...\n');
    
    % 可以通过修改rrt_star.m中的参数来测试不同场景
    % 这里我们创建一个修改版本来演示
    
    % 创建密集障碍物环境
    figure('Position', [200, 200, 800, 600]);
    
    % 模拟复杂环境的参数
    map_size = [0, 15, 0, 15];
    start_point = [1, 1];
    goal_point = [14, 14];
    
    % 更多障碍物
    obstacles = [
        3, 3, 1.0;
        5, 2, 0.8;
        7, 4, 1.2;
        4, 6, 0.9;
        8, 7, 1.1;
        6, 9, 1.0;
        10, 5, 1.3;
        9, 9, 0.8;
        11, 11, 1.0;
        12, 8, 0.9;
        2, 8, 0.8;
        3, 11, 1.0
    ];
    
    % 绘制复杂环境
    hold on;
    rectangle('Position', [map_size(1), map_size(3), ...
              map_size(2)-map_size(1), map_size(4)-map_size(3)], ...
              'EdgeColor', 'k', 'LineWidth', 2);
    
    for i = 1:size(obstacles, 1)
        center = obstacles(i, 1:2);
        radius = obstacles(i, 3);
        theta = linspace(0, 2*pi, 100);
        x_circle = center(1) + radius * cos(theta);
        y_circle = center(2) + radius * sin(theta);
        fill(x_circle, y_circle, [0.8, 0.8, 0.8], 'EdgeColor', 'k');
    end
    
    plot(start_point(1), start_point(2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot(goal_point(1), goal_point(2), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    axis equal;
    xlim([map_size(1)-1, map_size(2)+1]);
    ylim([map_size(3)-1, map_size(4)+1]);
    grid on;
    title('复杂环境示例 - 需要修改算法参数来处理', 'FontSize', 12);
    xlabel('X坐标'); ylabel('Y坐标');
    
    fprintf('✓ 复杂环境可视化完成\n');
    fprintf('  提示: 在实际应用中，需要根据环境复杂度调整算法参数\n');
    fprintf('  - 减小步长以提高精度\n');
    fprintf('  - 增加最大迭代次数\n');
    fprintf('  - 调整搜索半径\n');
end

function run_performance_analysis()
    % 性能分析测试
    fprintf('进行算法性能分析...\n');
    
    % 测试不同参数对性能的影响
    step_sizes = [0.5, 1.0, 1.5, 2.0];
    search_radii = [2.0, 3.0, 4.0, 5.0];
    
    fprintf('\n参数敏感性分析:\n');
    fprintf('%-10s %-15s %-15s %-15s\n', '步长', '搜索半径', '平均路径长度', '平均迭代次数');
    fprintf('%-10s %-15s %-15s %-15s\n', '----', '--------', '----------', '----------');
    
    % 注意：这里只是演示性能分析的框架
    % 实际测试需要多次运行算法并取平均值
    for i = 1:length(step_sizes)
        for j = 1:length(search_radii)
            % 模拟性能数据（实际应用中需要运行算法获得）
            avg_path_length = 20 + rand * 5;
            avg_iterations = 800 + round(rand * 400);
            
            fprintf('%-10.1f %-15.1f %-15.2f %-15d\n', ...
                step_sizes(i), search_radii(j), avg_path_length, avg_iterations);
        end
    end
    
    fprintf('\n性能优化建议:\n');
    fprintf('1. 步长越小，路径越精确，但需要更多迭代\n');
    fprintf('2. 搜索半径越大，路径质量越好，但计算开销越大\n');
    fprintf('3. 建议根据具体应用场景进行参数调优\n');
    
    % 创建性能比较图
    figure('Position', [300, 300, 1000, 400]);
    
    subplot(1, 2, 1);
    x_data = 1:4;
    y_data = [25.2, 23.8, 22.1, 21.5];  % 模拟不同步长的路径长度
    bar(x_data, y_data, 'FaceColor', [0.2, 0.6, 0.8]);
    set(gca, 'XTickLabel', {'0.5', '1.0', '1.5', '2.0'});
    xlabel('步长'); ylabel('平均路径长度');
    title('步长对路径质量的影响');
    grid on;
    
    subplot(1, 2, 2);
    y_data2 = [1200, 950, 850, 800];  % 模拟不同步长的迭代次数
    bar(x_data, y_data2, 'FaceColor', [0.8, 0.4, 0.2]);
    set(gca, 'XTickLabel', {'0.5', '1.0', '1.5', '2.0'});
    xlabel('步长'); ylabel('平均迭代次数');
    title('步长对收敛速度的影响');
    grid on;
    
    fprintf('✓ 性能分析图表已生成\n');
end