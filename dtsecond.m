clear; clc; close all;

H = 200;                     % 无人机高度（单位：m）
num_Tx = 48;                % 用户节点数量
area_size = 1200;           % 区域大小（1200×1200 平方米）
eta = 0.5;                  % SNR阈值
beta0 = 1e8;                % 信道系数
PA = 1000;                  % 用户节点发射功率 (mW)
Pu = 1000;                  % 无人机发射功率 (mW)
sigma0 = 2.2;               % 噪声功率谱密度参数
delta_A = PA / (sigma0^2);  % 发射端等效信噪比因子
delta_U = Pu / (sigma0^2);  % 无人机端等效信噪比因子

% 无人机参数
v_drone = 60 * 1000 / 3600; % 速度 (60 km/h = 16.67 m/s)
max_time = 800;             % 最大允许时间（秒）
min_throughput_req = 5.3;   % 最小吞吐量要求 (bps/Hz)

% ========== 物理信道衰减与小尺度衰落模型参数 ==========
eta_LoS = 1.0;                  % 视距 (LoS) 额外衰减 (dB)
eta_NLoS = 35.0;                % 非视距 (NLoS) 额外阴影/穿透损耗 (dB)
loss_LoS = 10^(-eta_LoS/10);    % 线性尺度下的 LoS 衰减乘子
loss_NLoS = 10^(-eta_NLoS/10);  % 线性尺度下的 NLoS 衰减乘子 
K_factor_dB = 10;               % LoS 链路的莱斯因子 (Rician K-factor, dB)
K_fac = 10^(K_factor_dB/10);    % 线性尺度下的莱斯因子

% ========== 实验扫参设定 ==========
fixed_re = 10;                % 彻底固定初始误差圆半径 (m)
rv_test_values = 10:2.5:60;   % 仅扫描视觉定位圆半径
num_tests = length(rv_test_values);

% 存储两大核心因变量的数组
outage_results = zeros(1, num_tests);
time_results = zeros(1, num_tests);


fprintf('========== 启动二维性能协同分析仿真 (修复命名冲突版) ==========\n');

%% ========== 1. 生成确定的 3D 建筑物与节点环境 ==========
num_buildings = 20;
buildings = zeros(num_buildings, 5); 
building_margin = 15; 

i = 1; max_retries = 2000; retries = 0;
while i <= num_buildings && retries < max_retries
    bx = rand() * (area_size - 150); by = rand() * (area_size - 150); 
    bw = 40 + rand() * 60; bl = 40 + rand() * 60; bh = 50 + rand() * 150;          
    overlap = false;
    for j = 1:(i-1)
        if ~(bx + bw + building_margin < buildings(j,1) || bx > buildings(j,1) + buildings(j,3) + building_margin || ...
             by + bl + building_margin < buildings(j,2) || by > buildings(j,2) + buildings(j,4) + building_margin)
            overlap = true; break;
        end
    end
    if ~overlap
        buildings(i, :) = [bx, by, bw, bl, bh];
        i = i + 1; retries = 0; 
    else
        retries = retries + 1;
    end
end
num_buildings = i - 1; 

Tx_nominal_pos = zeros(num_Tx, 2);
for i = 1:num_Tx
    valid_pos = false;
    while ~valid_pos
        temp_x = rand() * area_size; temp_y = rand() * area_size; is_overlap = false;
        for b = 1:num_buildings
            bx = buildings(b,1); by = buildings(b,2); bw = buildings(b,3); bl = buildings(b,4);
            if temp_x >= bx && temp_x <= bx + bw && temp_y >= by && temp_y <= by + bl
                is_overlap = true; break;
            end
        end
        if ~is_overlap
            Tx_nominal_pos(i, :) = [temp_x, temp_y]; valid_pos = true;
        end
    end
end 

epsilon_dbscan = 140; minPts = 3;
labels = dbscan(Tx_nominal_pos, epsilon_dbscan, minPts);
num_clusters = max(labels);
transfer_stations = zeros(num_clusters, 2);
cluster_node_indices = cell(num_clusters, 1);
for i = 1:num_clusters
    cluster_points = Tx_nominal_pos(labels == i, :);
    transfer_stations(i, :) = mean(cluster_points, 1);
    cluster_node_indices{i} = find(labels == i);
end
noise_indices = find(labels == -1);
if ~isempty(noise_indices)
    for i = 1:length(noise_indices)
        noise_point = Tx_nominal_pos(noise_indices(i), :);
        distances = sqrt(sum((transfer_stations - noise_point).^2, 2));
        [~, closest_cluster] = min(distances);
        labels(noise_indices(i)) = closest_cluster;
        cluster_node_indices{closest_cluster} = [cluster_node_indices{closest_cluster}; noise_indices(i)];
    end
    for i = 1:num_clusters
        cluster_points = Tx_nominal_pos(cluster_node_indices{i}, :);
        transfer_stations(i, :) = mean(cluster_points, 1);
    end
end

fprintf('环境初始化完成。正在执行参数扫描...\n\n');

%% ========== 2. 核心实验：遍历视觉定位圆半径 (r_v) ==========
for idx = 1:num_tests
    current_test_rv = rv_test_values(idx);
    fprintf('测试 r_v = %4.1f m ... ', current_test_rv);
    
    [outage, time_flown] = run_swarm_mission_dualmetric(...
        num_clusters, cluster_node_indices, transfer_stations, Tx_nominal_pos, ...
        fixed_re, current_test_rv, H, beta0, delta_A, delta_U, eta, v_drone, max_time, ...
        PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, min_throughput_req);
        
    outage_results(idx) = outage;
    time_results(idx) = time_flown;
    
    fprintf('中断=%.1f | 时间=%.1fs\n', outage, time_flown);
end

%% ========== 3. 数据平滑与自定义区间归一化 ==========
% 使用 LOESS 提取趋势，剥离震荡噪声
smooth_outage = smoothdata(outage_results, 'loess', 7);
smooth_time = smoothdata(time_results, 'loess', 7);

% 提取真实数值区间用于图例注释 (保留对原数据的追溯)
rng_outage = [min(smooth_outage), max(smooth_outage)];
rng_time = [min(smooth_time), max(smooth_time)];

% 映射到 [-0.7, 0.7] 之间
target_min = -0.7;
target_max = 0.7;
normalize = @(x) target_min + (target_max - target_min) * ((x - min(x)) / (max(x) - min(x)));

norm_outage = normalize(smooth_outage);
norm_time = normalize(smooth_time);

%% ========== 4. 绘制顶刊质感对比曲线 ==========
figure('Color', 'w', 'Position', [150, 150, 900, 650]); hold on;

% 定义学术感高级配色
color_outage = [0.8500, 0.1500, 0.1980]; % 深红 (代表代价/惩罚)
color_time   = [0.0000, 0.4470, 0.7410]; % 海蓝 (代表时间/持续)

% 绘制纯粹的平滑实线
plot(rv_test_values, norm_outage, '-', 'Color', color_outage, 'LineWidth', 3.5, 'DisplayName', 'Outage Cost (Penalty)');
plot(rv_test_values, norm_time, '-', 'Color', color_time, 'LineWidth', 3.5, 'DisplayName', 'Total Flight Time');

% 坐标系美化
grid on; grid minor;
ax = gca;
ax.GridColor = [0.6, 0.6, 0.6];
ax.GridAlpha = 0.3;
ax.MinorGridColor = [0.8, 0.8, 0.8];
ax.MinorGridAlpha = 0.2;
set(gca, 'FontSize', 14, 'LineWidth', 1.2, 'FontName', 'Arial');

% 将 Y 轴极限设为 [-0.85, 0.85]，保留呼吸感
ylim([-0.85, 0.85]); 

xlabel('Visual Locating Radius r_v (m)', 'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Normalized Performance Index [-0.7, 0.7]', 'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'Arial');
title('Performance Impact of Visual Radius (Outage vs. Time)', 'FontSize', 18, 'FontWeight', 'bold', 'FontName', 'Arial');

% 顶部图例
lgd = legend('Location', 'northeast', 'FontSize', 13, 'FontName', 'Arial');
lgd.EdgeColor = [0.7, 0.7, 0.7];
lgd.Color = 'w';

% 缩小版且放置于左下角的核心信息文本框
info_str = sprintf(['System Setup:\n' ...
    ' • Fixed r_e: %d m\n' ...
    ' • UAV Speed: %.1f m/s\n\n' ...
    'Raw Data Ranges:\n' ...
    ' \\color[rgb]{0.85,0.15,0.20}■\\color{black} Outage: %.1f ~ %.1f\n' ...
    ' \\color[rgb]{0.0,0.45,0.74}■\\color{black} Time: %.1f ~ %.1f s'], ...
    fixed_re, v_drone, rng_outage(1), rng_outage(2), rng_time(1), rng_time(2));

% 坐标调整至左下角: [x, y, width, height]
annotation('textbox', [0.145, 0.14, 0.25, 0.18], 'String', info_str, ...
    'FitBoxToText', 'on', 'BackgroundColor', [0.98, 0.98, 0.98], ...
    'EdgeColor', [0.5, 0.5, 0.5], 'LineWidth', 1.0, ...
    'FontSize', 10, 'FaceAlpha', 0.95, 'FontName', 'Arial', 'Interpreter', 'tex');

hold off;
fprintf('\n仿真实验全部完成，综合图表已生成！\n');


%% =========================================================================
%% 核心引擎: 支持双指标统计的严格 2 秒步长机制
%% =========================================================================
function [total_mission_outage, total_mission_time] = run_swarm_mission_dualmetric(...
    num_clusters, cluster_node_indices, transfer_stations, Tx_nominal_pos, ...
    test_error_radius, test_visual_radius, H, beta0, delta_A, delta_U, eta, v_drone, max_time, ...
    PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, min_throughput_req)

    min_error_radius = max(2, test_error_radius * 0.3); 
    lambda_decay = 0.1;
    
    % 【严谨恢复】：严格对齐到 2 秒高频判断
    detection_interval = 2;
    optimization_interval = 2;
    
    num_Tx = size(Tx_nominal_pos, 1);
    
    DT_user_nodes = struct('id', {}, 'nominal_pos', {}, 'true_pos', {}, 'error_circle', {}, 'visited', {}, 'detect_count', {});
    for i = 1:num_Tx
        DT_user_nodes(i).id = i;
        DT_user_nodes(i).nominal_pos = Tx_nominal_pos(i, :);
        DT_user_nodes(i).true_pos = Tx_nominal_pos(i, :);
        DT_user_nodes(i).error_circle = test_error_radius;
        DT_user_nodes(i).visited = false;
        DT_user_nodes(i).detect_count = 0;
    end

    total_mission_outage = 0;
    total_mission_time = 0;
    
    for cluster_idx = 1:num_clusters
        current_node_indices = cluster_node_indices{cluster_idx};
        num_nodes_in_cluster = length(current_node_indices);
        station_pos = transfer_stations(cluster_idx, :);
        
        all_cluster_positions = zeros(num_nodes_in_cluster, 2);
        for i = 1:num_nodes_in_cluster
            all_cluster_positions(i, :) = DT_user_nodes(current_node_indices(i)).nominal_pos;
        end
        
        current_uav_pos = station_pos;
        visited_nodes = false(num_nodes_in_cluster, 1);
        
        total_flight_time = 0; 
        total_cluster_outage = 0; 
        time_since_last_optimization = 0; current_target_idx = []; 
        
        while sum(visited_nodes) < num_nodes_in_cluster && total_flight_time < max_time
            if isempty(current_target_idx) || time_since_last_optimization >= optimization_interval
                unvisited_indices = find(~visited_nodes);
                num_unvisited = length(unvisited_indices);
                detected_positions = zeros(num_unvisited, 2);
                
                for i = 1:num_unvisited
                    local_idx = unvisited_indices(i);
                    global_idx = current_node_indices(local_idx);
                    nominal_pos = DT_user_nodes(global_idx).nominal_pos;
                    
                    DT_user_nodes(global_idx).detect_count = DT_user_nodes(global_idx).detect_count + 1;
                    current_re = max(min_error_radius, test_error_radius * exp(-lambda_decay * DT_user_nodes(global_idx).detect_count));
                    DT_user_nodes(global_idx).error_circle = current_re;
                    
                    angle = rand() * 2 * pi;
                    r = sqrt(rand()) * current_re;
                    detected_positions(i, :) = nominal_pos + [r * cos(angle), r * sin(angle)];
                end
                
                [optimized_path, ~, ~, ~, ~] = ...
                    optimize_path_vectorized_LUT(current_uav_pos, station_pos, ...
                    detected_positions, H, beta0, delta_A, delta_U, eta, v_drone, ...
                    max_time - total_flight_time, min_throughput_req, PA, Pu, sigma0, ...
                    loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions); 
                
                if isempty(optimized_path)
                    distances_to_unvisited = sqrt(sum((detected_positions - current_uav_pos).^2, 2));
                    [~, nearest_idx] = min(distances_to_unvisited);
                    optimized_path = nearest_idx;
                end
                current_target_idx = unvisited_indices(optimized_path(1));
                time_since_last_optimization = 0;  
            end
            
            next_global_idx = current_node_indices(current_target_idx);
            target_nominal_pos = DT_user_nodes(next_global_idx).nominal_pos;
            
            DT_user_nodes(next_global_idx).detect_count = DT_user_nodes(next_global_idx).detect_count + 1;
            current_re = max(min_error_radius, test_error_radius * exp(-lambda_decay * DT_user_nodes(next_global_idx).detect_count));
            DT_user_nodes(next_global_idx).error_circle = current_re;
            
            angle = rand() * 2 * pi;
            r = sqrt(rand()) * current_re;
            current_detected_pos = target_nominal_pos + [r * cos(angle), r * sin(angle)];
            
            dist_to_detected = norm(current_detected_pos - current_uav_pos);
            
            time_step = min(detection_interval, dist_to_detected / v_drone);
            
            direction = current_detected_pos - current_uav_pos;
            if norm(direction) > 1e-6, direction = direction / norm(direction); else, direction = [0, 0]; end
            fly_distance = v_drone * time_step;
            new_pos = current_uav_pos + direction * fly_distance;
            
            P1 = current_uav_pos; d_vec = new_pos - P1; a_quad = dot(d_vec, d_vec);
            hit_t = inf; hit_global_idx = -1; hit_local_idx = -1;
            
            unvisited_indices = find(~visited_nodes);
            for i = 1:length(unvisited_indices)
                loc_idx = unvisited_indices(i);
                glob_idx = current_node_indices(loc_idx);
                C = DT_user_nodes(glob_idx).nominal_pos;
                f_vec = P1 - C;
                b_quad = 2 * dot(f_vec, d_vec);
                c_quad = dot(f_vec, f_vec) - test_visual_radius^2;
                
                if a_quad > 1e-6
                    discriminant = b_quad^2 - 4 * a_quad * c_quad;
                    if discriminant >= 0
                        t1 = (-b_quad - sqrt(discriminant)) / (2 * a_quad);
                        t2 = (-b_quad + sqrt(discriminant)) / (2 * a_quad);
                        valid_t = [];
                        if t1 >= -1e-6 && t1 <= 1 + 1e-6, valid_t = [valid_t, max(0, t1)]; end
                        if t2 >= -1e-6 && t2 <= 1 + 1e-6, valid_t = [valid_t, max(0, t2)]; end
                        if c_quad <= 0, valid_t = [valid_t, 0]; end
                        if ~isempty(valid_t)
                            min_t = min(valid_t);
                            if min_t < hit_t
                                hit_t = min_t; hit_global_idx = glob_idx; hit_local_idx = loc_idx;
                            end
                        end
                    end
                else
                    if c_quad <= 0 && 0 < hit_t
                        hit_t = 0; hit_global_idx = glob_idx; hit_local_idx = loc_idx;
                    end
                end
            end
            
            if hit_t <= 1.0
                hit_pos = P1 + hit_t * d_vec;
                actual_fly_dist = fly_distance * hit_t;
                actual_time_step = time_step * hit_t;
                
                if actual_fly_dist > 1e-3
                    [seg_outage, ~] = calculate_segment_metrics_vectorized(current_uav_pos, hit_pos, all_cluster_positions, station_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
                    total_cluster_outage = total_cluster_outage + seg_outage;
                    total_flight_time = total_flight_time + actual_time_step;
                end
                
                current_uav_pos = DT_user_nodes(hit_global_idx).true_pos;
                visited_nodes(hit_local_idx) = true;
                DT_user_nodes(hit_global_idx).visited = true;
                current_target_idx = []; 
                time_since_last_optimization = optimization_interval; 
            else
                [seg_outage, ~] = calculate_segment_metrics_vectorized(current_uav_pos, new_pos, all_cluster_positions, station_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
                total_cluster_outage = total_cluster_outage + seg_outage;
                total_flight_time = total_flight_time + time_step;
                time_since_last_optimization = time_since_last_optimization + time_step;
                current_uav_pos = new_pos;
            end
            if total_flight_time > max_time * 0.95, break; end
        end
        
        return_complete = false;
        while ~return_complete
            distance_to_station = norm(current_uav_pos - station_pos);
            if distance_to_station < 1.0
                return_complete = true;
            else
                time_step = min(detection_interval, distance_to_station / v_drone);
                direction = station_pos - current_uav_pos;
                direction = direction / norm(direction);
                fly_distance = min(v_drone * time_step, distance_to_station);
                new_pos = current_uav_pos + direction * fly_distance;
                [seg_outage, ~] = calculate_segment_metrics_vectorized(current_uav_pos, new_pos, all_cluster_positions, station_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
                total_cluster_outage = total_cluster_outage + seg_outage;
                total_flight_time = total_flight_time + time_step;
                current_uav_pos = new_pos;
            end
        end
        
        total_mission_outage = total_mission_outage + total_cluster_outage;
        total_mission_time = total_mission_time + total_flight_time;
    end
end

%% ================== 底层信道与遮挡物理评估 (保留向量化算法) ==================
function blocked = is_blocked_by_buildings_vectorized(p1, p2, buildings)
    d = p2(:) - p1(:);
    if norm(d) < 1e-6, blocked = false; return; end
    n_b = size(buildings, 1);
    x_min = buildings(:,1); x_max = x_min + buildings(:,3);
    y_min = buildings(:,2); y_max = y_min + buildings(:,4); bh = buildings(:,5);
    dx = d(1); dy = d(2); dz = d(3);
    
    if abs(dx) < 1e-6
        t_x_min = zeros(n_b, 1); t_x_max = ones(n_b, 1);
        out_bounds = p1(1) < x_min | p1(1) > x_max;
        t_x_min(out_bounds) = inf; t_x_max(out_bounds) = -inf;
    else
        t1 = (x_min - p1(1)) / dx; t2 = (x_max - p1(1)) / dx;
        t_x_min = min(t1, t2); t_x_max = max(t1, t2);
    end
    if abs(dy) < 1e-6
        t_y_min = zeros(n_b, 1); t_y_max = ones(n_b, 1);
        out_bounds = p1(2) < y_min | p1(2) > y_max;
        t_y_min(out_bounds) = inf; t_y_max(out_bounds) = -inf;
    else
        t3 = (y_min - p1(2)) / dy; t4 = (y_max - p1(2)) / dy;
        t_y_min = min(t3, t4); t_y_max = max(t3, t4);
    end
    t_low = max([t_x_min, t_y_min, zeros(n_b, 1)], [], 2);
    t_high = min([t_x_max, t_y_max, ones(n_b, 1)], [], 2);
    
    valid = t_low <= t_high + 1e-6;
    if any(valid)
        t_mid = (t_low(valid) + t_high(valid)) / 2;
        z_mid = p1(3) + t_mid * dz;
        if any(z_mid <= bh(valid)), blocked = true; return; end
    end
    blocked = false;
end

function [outage_cost, throughput] = calculate_segment_metrics_vectorized(pos1, pos2, all_targets, station_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac)
    mid_pos = (pos1 + pos2) / 2;
    pos_UAV = [mid_pos(1), mid_pos(2), H]; pos_station = [station_pos(1), station_pos(2), 0];
    d_rx_dist = max(0.1, norm(pos_UAV - pos_station)); 
    if is_blocked_by_buildings_vectorized(pos_UAV, pos_station, buildings)
        beta2_eff = beta0 * loss_NLoS / (d_rx_dist^2); 
        gamma_bar2 = beta2_eff * delta_U; P_succ_2 = exp(-eta / gamma_bar2); 
    else
        beta2_eff = beta0 * loss_LoS / (d_rx_dist^2);  
        gamma_bar2 = beta2_eff * delta_U;
        a2 = sqrt(2 * K_fac); b2 = sqrt(2 * (K_fac + 1) * eta / gamma_bar2); P_succ_2 = my_marcumq(a2, b2);     
    end
    if isempty(all_targets)
        outage_cost = 0; throughput = log2(1 + gamma_bar2); return;
    end
    max_outage = 0; min_throughput = inf; 
    for i = 1:size(all_targets, 1)
        pos_tx = [all_targets(i,1), all_targets(i,2), 0]; 
        d_tx_dist = max(0.1, norm(pos_UAV - pos_tx));
        if is_blocked_by_buildings_vectorized(pos_UAV, pos_tx, buildings)
            beta1_eff = beta0 * loss_NLoS / (d_tx_dist^2); 
            gamma_bar1 = beta1_eff * delta_A; P_succ_1 = exp(-eta / gamma_bar1); 
        else
            beta1_eff = beta0 * loss_LoS / (d_tx_dist^2); 
            gamma_bar1 = beta1_eff * delta_A;
            a1 = sqrt(2 * K_fac); b1 = sqrt(2 * (K_fac + 1) * eta / gamma_bar1); P_succ_1 = my_marcumq(a1, b1);     
        end
        p_out = 1 - (P_succ_1 * P_succ_2);
        if p_out > max_outage, max_outage = p_out; end
        curr_th = log2(1 + min(gamma_bar1, gamma_bar2));
        if curr_th < min_throughput, min_throughput = curr_th; end
    end
    outage_cost = max_outage * norm(pos2 - pos1); throughput = min_throughput;
end

function [best_path, total_distance, total_outage, min_throughput, best_fitness] = optimize_path_vectorized_LUT(current_pos, return_pos, target_positions, H, beta0, delta_A, delta_U, eta, v_drone, remaining_time, min_throughput_req, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions)
    num_targets = size(target_positions, 1);
    if num_targets == 0, best_path = []; total_distance = 0; total_outage = 0; min_throughput = inf; best_fitness = 0; return; end
    if num_targets == 1
        best_path = 1; total_distance = norm(current_pos - target_positions(1,:)) + norm(target_positions(1,:) - return_pos);
        [total_outage, min_throughput] = calculate_segment_metrics_vectorized(current_pos, target_positions, all_cluster_positions, return_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
        best_fitness = 0; return;
    end
    N_all = num_targets + 2; all_pts = [current_pos; target_positions; return_pos]; 
    dist_mat = zeros(N_all, N_all); outage_mat = zeros(N_all, N_all); throughput_mat = inf(N_all, N_all);
    for i = 1:N_all
        for j = 1:N_all
            if i ~= j
                dist_mat(i,j) = norm(all_pts(i,:) - all_pts(j,:));
                [out_val, th_val] = calculate_segment_metrics_vectorized(all_pts(i,:), all_pts(j,:), all_cluster_positions, return_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
                outage_mat(i,j) = out_val; throughput_mat(i,j) = th_val;
            end
        end
    end
    
    % GA参数完全对齐 dtrvoutage.m
    pop_size = min(40, max(20, num_targets * 4)); max_generations = min(50, max(30, num_targets * 4));
    crossover_rate = 0.8; mutation_rate = 0.2; elite_count = max(2, round(pop_size * 0.1));
    
    population = cell(pop_size, 1); for i = 1:pop_size, population{i} = randperm(num_targets); end
    fitness = zeros(pop_size, 1); for i = 1:pop_size, fitness(i) = evaluate_fitness_LUT_vectorized(population{i}, dist_mat, outage_mat, throughput_mat, v_drone, remaining_time, min_throughput_req); end
    for gen = 1:max_generations
        [selected_pop, ~] = selection_operation(population, fitness, pop_size);
        offspring_pop = crossover_operation(selected_pop, crossover_rate, num_targets);
        mutated_pop = mutation_operation(offspring_pop, mutation_rate, num_targets);
        new_fitness = zeros(pop_size, 1); for i = 1:pop_size, new_fitness(i) = evaluate_fitness_LUT_vectorized(mutated_pop{i}, dist_mat, outage_mat, throughput_mat, v_drone, remaining_time, min_throughput_req); end
        [population, fitness] = elitism_operation(population, fitness, mutated_pop, new_fitness, elite_count);
    end
    [best_fitness, best_idx] = min(fitness); best_path = population{best_idx};
    full_path = [1, best_path + 1, N_all]; u = full_path(1:end-1); v = full_path(2:end); idx = u + (v - 1) * N_all;
    total_distance = sum(dist_mat(idx)); total_outage = sum(outage_mat(idx)); min_throughput = min(throughput_mat(idx));
end

function fitness = evaluate_fitness_LUT_vectorized(path, dist_mat, outage_mat, throughput_mat, v_drone, max_time, min_throughput_req)
    N_all = size(dist_mat, 1); full_path = [1, path + 1, N_all]; u = full_path(1:end-1); v = full_path(2:end); idx = u + (v - 1) * N_all; 
    total_dist = sum(dist_mat(idx)); total_outage = sum(outage_mat(idx)); min_th = min(throughput_mat(idx));
    total_time = total_dist / v_drone; time_penalty = 0; if total_time > max_time, time_penalty = 1000 * (total_time - max_time); end
    throughput_term = 0; if min_th < min_throughput_req, throughput_term = 1000 * (min_throughput_req - min_th); else, throughput_term = -50 * log(1 + (min_th - min_throughput_req)); end
    distance_weight = 0.2; throughput_weight = 0.2; outage_weight = 0.6;
    fitness = distance_weight * (total_dist / 1000) + throughput_weight * throughput_term + outage_weight * (total_outage / 100) + time_penalty;
end

% 【核心修复区域】：强制调用内置函数 ncx2cdf 或 integral，杜绝命名冲突
function q = my_marcumq(a, b)
    if ~isempty(which('ncx2cdf'))
        q = 1 - ncx2cdf(b.^2, 2, a.^2); 
    else
        f = @(x) x .* exp(-(x.^2 + a.^2)/2) .* besseli(0, a.*x); 
        q = integral(f, b, inf); 
    end
end

function [selected_pop, selected_fit] = selection_operation(population, fitness, pop_size)
    selected_pop = cell(pop_size, 1); selected_fit = zeros(pop_size, 1); tournament_size = 3;
    for i = 1:pop_size
        candidates = randperm(length(population), min(tournament_size, length(population)));
        [best_fit, best_idx] = min(fitness(candidates)); selected_pop{i} = population{candidates(best_idx)}; selected_fit(i) = best_fit;
    end
end
function offspring_pop = crossover_operation(parent_pop, crossover_rate, num_genes)
    offspring_pop = parent_pop; num_parents = length(parent_pop);
    for i = 1:2:num_parents-1
        if rand < crossover_rate
            parent1 = parent_pop{i}; parent2 = parent_pop{i+1}; point1 = randi([1, num_genes-1]); point2 = randi([point1+1, num_genes]); child = zeros(1, num_genes); child(point1:point2) = parent1(point1:point2); idx = 1;
            for j = 1:num_genes
                if ~ismember(parent2(j), child(point1:point2))
                    while idx >= point1 && idx <= point2, idx = idx + 1; end
                    if idx <= num_genes, child(idx) = parent2(j); idx = idx + 1; end
                end
            end
            offspring_pop{i} = child;
        end
    end
end
function mutated_pop = mutation_operation(population, mutation_rate, num_genes)
    mutated_pop = population; num_pop = length(population);
    for i = 1:num_pop
        if rand < mutation_rate
            individual = population{i};
            if num_genes >= 2
                pos1 = randi([1, num_genes]); pos2 = randi([1, num_genes]);
                while pos1 == pos2, pos2 = randi([1, num_genes]); end
                temp = individual(pos1); individual(pos1) = individual(pos2); individual(pos2) = temp;
            end
            mutated_pop{i} = individual;
        end
    end
end
function [new_pop, new_fit] = elitism_operation(old_pop, old_fit, offspring_pop, offspring_fit, elite_count)
    combined_pop = [old_pop; offspring_pop]; combined_fit = [old_fit; offspring_fit]; [sorted_fit, sorted_idx] = sort(combined_fit);
    new_pop = combined_pop(sorted_idx(1:length(old_pop))); new_fit = sorted_fit(1:length(old_pop));
end
function labels = dbscan(X, epsilon, minPts)
    n = size(X, 1); labels = zeros(n, 1); visited = false(n, 1); clusterId = 0;
    for i = 1:n
        if ~visited(i)
            visited(i) = true; neighbors = regionQuery(X, i, epsilon);
            if length(neighbors) < minPts, labels(i) = -1; else
                clusterId = clusterId + 1; labels(i) = clusterId; k = 1;
                while k <= length(neighbors)
                    point = neighbors(k);
                    if ~visited(point)
                        visited(point) = true; newNeighbors = regionQuery(X, point, epsilon);
                        if length(newNeighbors) >= minPts, neighbors = [neighbors; newNeighbors]; end
                    end
                    if labels(point) == 0 || labels(point) == -1, labels(point) = clusterId; end
                    k = k + 1;
                end
            end
        end
    end
end
function neighbors = regionQuery(X, pointIdx, epsilon)
    distances = sqrt(sum((X - X(pointIdx, :)).^2, 2)); neighbors = find(distances <= epsilon & distances > 0);
end