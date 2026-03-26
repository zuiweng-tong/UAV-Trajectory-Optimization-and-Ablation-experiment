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

% ========== 位置不确定性参数 ==========
error_radius = 15;          % 误差圆半径 (m)
min_error_radius = 5;      % 最小误差圆半径定值 (m)
lambda_decay = 0.1;         % 误差圆衰减系数 (\lambda)
visual_radius = 38;         % 视觉定位圆半径 (m)
detection_interval = 5;     % 飞行中位置检测间隔 (s)
optimization_interval = 5;  % 轨迹优化间隔 (s) 

% ========== 物理信道衰减与小尺度衰落模型参数 ==========
eta_LoS = 1.0;                  % 视距 (LoS) 额外衰减 (dB)
eta_NLoS = 35.0;                % 非视距 (NLoS) 额外阴影/穿透损耗 (dB)
loss_LoS = 10^(-eta_LoS/10);    % 线性尺度下的 LoS 衰减乘子
loss_NLoS = 10^(-eta_NLoS/10);  % 线性尺度下的 NLoS 衰减乘子 

K_factor_dB = 10;               % LoS 链路的莱斯因子 (Rician K-factor, dB)
K_fac = 10^(K_factor_dB/10);    % 线性尺度下的莱斯因子



%% ========== 生成不重叠的 3D 建筑物 ==========
num_buildings = 20;
buildings = zeros(num_buildings, 5); % [x, y, width, length, height]
building_margin = 15; % 建筑物之间的最小间距 (m)

i = 1;
max_retries = 2000; % 防止死循环
retries = 0;

while i <= num_buildings && retries < max_retries
    bx = rand() * (area_size - 150); 
    by = rand() * (area_size - 150); 
    bw = 40 + rand() * 60;           
    bl = 40 + rand() * 60   ;           
    bh = 50 + rand() * 150;          
    
    overlap = false;
    for j = 1:(i-1)
        if ~(bx + bw + building_margin < buildings(j,1) || ...
             bx > buildings(j,1) + buildings(j,3) + building_margin || ...
             by + bl + building_margin < buildings(j,2) || ...
             by > buildings(j,2) + buildings(j,4) + building_margin)
            overlap = true;
            break;
        end
    end
    
    if ~overlap
        buildings(i, :) = [bx, by, bw, bl, bh];
        i = i + 1;
        retries = 0; 
    else
        retries = retries + 1;
    end
end
num_buildings = i - 1; 

%% ================== 初始化用户节点位置 ==================
% 避免节点生成在建筑物内部
Tx_nominal_pos = zeros(num_Tx, 2);
for i = 1:num_Tx
    valid_pos = false;
    while ~valid_pos
        temp_x = rand() * area_size;
        temp_y = rand() * area_size;
        is_overlap = false;
        
        for b = 1:num_buildings
            bx = buildings(b,1); by = buildings(b,2); 
            bw = buildings(b,3); bl = buildings(b,4);
            if temp_x >= bx && temp_x <= bx + bw && temp_y >= by && temp_y <= by + bl
                is_overlap = true;
                break;
            end
        end
        
        if ~is_overlap
            Tx_nominal_pos(i, :) = [temp_x, temp_y];
            valid_pos = true;
        end
    end
end 

DT_user_nodes = struct('id', {}, 'nominal_pos', {}, 'true_pos', {}, ...
                       'error_circle', {}, 'visual_circle', {}, 'visited', {}, 'detect_count', {});

for i = 1:num_Tx
    DT_user_nodes(i).id = i;
    DT_user_nodes(i).nominal_pos = Tx_nominal_pos(i, :);
    DT_user_nodes(i).true_pos = Tx_nominal_pos(i, :);
    DT_user_nodes(i).error_circle = error_radius;
    DT_user_nodes(i).visual_circle = visual_radius;
    DT_user_nodes(i).visited = false;
    DT_user_nodes(i).detect_count = 0;
end

fprintf('========== 数字孪生系统初始化完成 ==========\n');
fprintf('生成了 %d 栋互不重叠的大楼。\n', num_buildings);

%% ================== DBSCAN聚类与零散节点处理 ==================
epsilon = 140;
minPts = 3;

labels = dbscan(Tx_nominal_pos, epsilon, minPts);
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

%% ================== 为每个聚类执行UAV任务 ==================
cluster_results = cell(num_clusters, 1);
all_trajectories = cell(num_clusters, 1);

for cluster_idx = 1:num_clusters
    current_node_indices = cluster_node_indices{cluster_idx};
    num_nodes_in_cluster = length(current_node_indices);
    station_pos = transfer_stations(cluster_idx, :);
    
    all_cluster_positions = zeros(num_nodes_in_cluster, 2);
    for i = 1:num_nodes_in_cluster
        all_cluster_positions(i, :) = DT_user_nodes(current_node_indices(i)).nominal_pos;
    end
    
    segments = cell(0, 1);
    current_segment = station_pos;
    current_uav_pos = station_pos;
    visited_nodes = false(num_nodes_in_cluster, 1);
    visit_sequence = [];
    
    total_flight_time = 0; total_flight_distance = 0;
    total_outage_cost = 0; min_throughput_all = inf;
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
                current_re = max(min_error_radius, error_radius * exp(-lambda_decay * DT_user_nodes(global_idx).detect_count));
                DT_user_nodes(global_idx).error_circle = current_re;
                
                angle = rand() * 2 * pi;
                r = sqrt(rand()) * current_re;
                detected_positions(i, :) = nominal_pos + [r * cos(angle), r * sin(angle)];
            end
            
            [optimized_path, opt_distance, opt_outage, opt_throughput, ~] = ...
                optimize_path_from_current_position(current_uav_pos, station_pos, ...
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
        
        % 全局几何射线碰撞检测
        next_global_idx = current_node_indices(current_target_idx);
        target_nominal_pos = DT_user_nodes(next_global_idx).nominal_pos;
        
        DT_user_nodes(next_global_idx).detect_count = DT_user_nodes(next_global_idx).detect_count + 1;
        current_re = max(min_error_radius, error_radius * exp(-lambda_decay * DT_user_nodes(next_global_idx).detect_count));
        DT_user_nodes(next_global_idx).error_circle = current_re;
        
        angle = rand() * 2 * pi;
        r = sqrt(rand()) * current_re;
        current_detected_pos = target_nominal_pos + [r * cos(angle), r * sin(angle)];
        
        dist_to_detected = norm(current_detected_pos - current_uav_pos);
        time_step = min(detection_interval, dist_to_detected / v_drone);
        
        direction = current_detected_pos - current_uav_pos;
        if norm(direction) > 1e-6
            direction = direction / norm(direction);
        else
            direction = [0, 0];
        end
        
        fly_distance = v_drone * time_step;
        new_pos = current_uav_pos + direction * fly_distance;
        
        % 用线段去与所有未访问节点的视觉圆做相交测试
        P1 = current_uav_pos;
        d_vec = new_pos - P1;
        a_quad = dot(d_vec, d_vec);
        
        hit_t = inf;
        hit_global_idx = -1;
        hit_local_idx = -1;
        
        unvisited_indices = find(~visited_nodes);
        for i = 1:length(unvisited_indices)
            loc_idx = unvisited_indices(i);
            glob_idx = current_node_indices(loc_idx);
            C = DT_user_nodes(glob_idx).nominal_pos;
            
            f_vec = P1 - C;
            b_quad = 2 * dot(f_vec, d_vec);
            c_quad = dot(f_vec, f_vec) - visual_radius^2;
            
            if a_quad > 1e-6
                discriminant = b_quad^2 - 4 * a_quad * c_quad;
                if discriminant >= 0
                    t1 = (-b_quad - sqrt(discriminant)) / (2 * a_quad);
                    t2 = (-b_quad + sqrt(discriminant)) / (2 * a_quad);
                    
                    valid_t = [];
                    if t1 >= -1e-6 && t1 <= 1 + 1e-6, valid_t = [valid_t, max(0, t1)]; end
                    if t2 >= -1e-6 && t2 <= 1 + 1e-6, valid_t = [valid_t, max(0, t2)]; end
                    
                    if c_quad <= 0 
                        valid_t = [valid_t, 0];
                    end
                    
                    if ~isempty(valid_t)
                        min_t = min(valid_t);
                        if min_t < hit_t
                            hit_t = min_t;
                            hit_global_idx = glob_idx;
                            hit_local_idx = loc_idx;
                        end
                    end
                end
            else
                if c_quad <= 0
                    if 0 < hit_t
                        hit_t = 0;
                        hit_global_idx = glob_idx;
                        hit_local_idx = loc_idx;
                    end
                end
            end
        end
        
        if hit_t <= 1.0
            hit_pos = P1 + hit_t * d_vec;
            actual_fly_dist = fly_distance * hit_t;
            actual_time_step = time_step * hit_t;
            
            if actual_fly_dist > 1e-3
                current_segment = [current_segment; hit_pos];
                [seg_outage, seg_throughput] = calculate_segment_metrics(...
                    current_uav_pos, hit_pos, all_cluster_positions, station_pos, ...
                    H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, ...
                    loss_LoS, loss_NLoS, buildings, K_fac);
                
                total_outage_cost = total_outage_cost + seg_outage;
                min_throughput_all = min(min_throughput_all, seg_throughput);
                total_flight_distance = total_flight_distance + actual_fly_dist;
                total_flight_time = total_flight_time + actual_time_step;
            end
            
            target_true_pos = DT_user_nodes(hit_global_idx).true_pos;
            segments{end+1} = current_segment;
            current_uav_pos = target_true_pos;
            current_segment = target_true_pos;
            visited_nodes(hit_local_idx) = true;
            DT_user_nodes(hit_global_idx).visited = true;
            visit_sequence = [visit_sequence; hit_global_idx];
            
            current_target_idx = []; 
            time_since_last_optimization = optimization_interval; 
        else
            current_segment = [current_segment; new_pos];
            [seg_outage, seg_throughput] = calculate_segment_metrics(...
                current_uav_pos, new_pos, all_cluster_positions, station_pos, ...
                H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, ...
                loss_LoS, loss_NLoS, buildings, K_fac);
            
            total_outage_cost = total_outage_cost + seg_outage;
            min_throughput_all = min(min_throughput_all, seg_throughput);
            total_flight_distance = total_flight_distance + fly_distance;
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
            current_segment = [current_segment; station_pos];
            segments{end+1} = current_segment;
            return_complete = true;
        else
            time_step = min(detection_interval, distance_to_station / v_drone);
            direction = station_pos - current_uav_pos;
            direction = direction / norm(direction);
            fly_distance = min(v_drone * time_step, distance_to_station);
            new_pos = current_uav_pos + direction * fly_distance;
            current_segment = [current_segment; new_pos];
            
            [seg_outage, seg_throughput] = calculate_segment_metrics(...
                current_uav_pos, new_pos, all_cluster_positions, station_pos, ...
                H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, ...
                loss_LoS, loss_NLoS, buildings, K_fac);
            
            total_outage_cost = total_outage_cost + seg_outage;
            min_throughput_all = min(min_throughput_all, seg_throughput);
            total_flight_distance = total_flight_distance + fly_distance;
            total_flight_time = total_flight_time + time_step;
            current_uav_pos = new_pos;
        end
    end
    
    cluster_results{cluster_idx} = struct('total_time', total_flight_time, 'total_distance', total_flight_distance, ...
        'total_outage', total_outage_cost, 'nodes_visited', sum(visited_nodes), 'nodes_total', num_nodes_in_cluster);
    all_trajectories{cluster_idx} = segments;
    fprintf('Cluster %d 轨迹规划完成。\n', cluster_idx);
end

%% ================== 总体 3D 轨迹与环境可视化 ==================
figure('Color', 'w', 'Position', [100, 100, 1200, 1000]); hold on;
view(3); grid on;

for i = 1:num_buildings
    bx = buildings(i,1); by = buildings(i,2); 
    bw = buildings(i,3); bl = buildings(i,4); bh = buildings(i,5);
    draw_3d_building(bx, by, bw, bl, bh);
end

cluster_colors = lines(num_clusters);
theta_circ = linspace(0, 2*pi, 50); 

for cluster_idx = 1:num_clusters
    current_node_indices = cluster_node_indices{cluster_idx};
    
    for i = 1:length(current_node_indices)
        pos = DT_user_nodes(current_node_indices(i)).nominal_pos;
        curr_r = DT_user_nodes(current_node_indices(i)).error_circle;
        
        plot3(pos(1) + curr_r*cos(theta_circ), pos(2) + curr_r*sin(theta_circ), zeros(1,50), 'r--', 'LineWidth', 1.0);
        plot3(pos(1) + visual_radius*cos(theta_circ), pos(2) + visual_radius*sin(theta_circ), zeros(1,50), 'b:', 'LineWidth', 1.0);
        plot3(pos(1), pos(2), 0, 'o', 'MarkerSize', 6, 'MarkerFaceColor', cluster_colors(cluster_idx,:), 'MarkerEdgeColor', 'k');
    end
    
    station_pos = transfer_stations(cluster_idx, :);
    plot3(station_pos(1), station_pos(2), 0, '^', 'MarkerSize', 15, 'MarkerFaceColor', cluster_colors(cluster_idx,:), 'MarkerEdgeColor', 'k');
    
    segments = all_trajectories{cluster_idx};
    for seg_idx = 1:length(segments)
        if size(segments{seg_idx}, 1) >= 2
            seg_x = segments{seg_idx}(:,1);
            seg_y = segments{seg_idx}(:,2);
            seg_z = repmat(H, length(seg_x), 1); 
            plot3(seg_x, seg_y, seg_z, '-', 'Color', cluster_colors(cluster_idx,:), 'LineWidth', 3.0);
        end
    end
    
    if ~isempty(segments) && size(segments{1}, 1) >= 1
        start_p = segments{1}(1,:);
        plot3([station_pos(1), start_p(1)], [station_pos(2), start_p(2)], [0, H], ':', 'Color', cluster_colors(cluster_idx,:), 'LineWidth', 1.5);
    end
end

h_err = plot3(NaN, NaN, NaN, 'r--', 'LineWidth', 1.5);
h_vis = plot3(NaN, NaN, NaN, 'b:', 'LineWidth', 1.5);
legend([h_err, h_vis], {'Error Circle (Dynamic)', 'Visual Circle (38m)'}, 'Location', 'northeast', 'FontSize', 10);

xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
zlabel('Z (Altitude, m)', 'FontSize', 12, 'FontWeight', 'bold');
title('3D UAV Trajectories and Urban Digital Twin Environment', 'FontSize', 16, 'FontWeight', 'bold');
xlim([0 area_size]); ylim([0 area_size]); zlim([0 H+100]);
set(gca, 'FontSize', 12);
view(-35, 45); 
hold off;

%% ================== 最大聚类中断概率热力图 (极速平滑版) ==================
cluster_sizes = cellfun(@length, cluster_node_indices);
[~, largest_cluster_idx] = max(cluster_sizes);
fprintf('\n========== 生成节点最多的聚类 (第 %d 个) 的光线追踪热力图 ==========\n', largest_cluster_idx);

tx_global_idx = cluster_node_indices{largest_cluster_idx};
num_tx_in_cluster = length(tx_global_idx);
tx_positions = zeros(num_tx_in_cluster, 2);
for i = 1:num_tx_in_cluster
    tx_positions(i, :) = DT_user_nodes(tx_global_idx(i)).nominal_pos;
end

rx_position = transfer_stations(largest_cluster_idx, :);

margin_hot = 100; 
min_x = max(0, min(tx_positions(:,1)) - margin_hot);
max_x = min(area_size, max(tx_positions(:,1)) + margin_hot);
min_y = max(0, min(tx_positions(:,2)) - margin_hot);
max_y = min(area_size, max(tx_positions(:,2)) + margin_hot);

% 【极速优化2】：恢复较大的物理计算步长，极大缩减运算量
step_calc = 8;  
x_vec = min_x:step_calc:max_x;
y_vec = min_y:step_calc:max_y;
[X_grid, Y_grid] = meshgrid(x_vec, y_vec);
P_outage = zeros(size(X_grid));

for i = 1:size(X_grid,1)
    for j = 1:size(X_grid,2)
        x = X_grid(i,j); y = Y_grid(i,j);
        max_p = 0;
        uav_pos_3d = [x, y, H];
        
        rx_pos_3d = [rx_position, 0];
        d_rx = max(0.1, norm(uav_pos_3d - rx_pos_3d));
        
        if is_blocked_by_buildings(uav_pos_3d, rx_pos_3d, buildings)
            beta2_eff = beta0 * loss_NLoS / (d_rx^2); 
            gamma_bar2 = beta2_eff * delta_U;
            P_succ_2 = exp(-eta / gamma_bar2); 
        else
            beta2_eff = beta0 * loss_LoS / (d_rx^2);  
            gamma_bar2 = beta2_eff * delta_U;
            a2 = sqrt(2 * K_fac);
            b2 = sqrt(2 * (K_fac + 1) * eta / gamma_bar2);
            P_succ_2 = my_marcumq(a2, b2);     
        end
        
        for k = 1:num_tx_in_cluster
            user_pos_3d = [tx_positions(k, :), 0];
            d_tx = max(0.1, norm(uav_pos_3d - user_pos_3d));
            
            if is_blocked_by_buildings(uav_pos_3d, user_pos_3d, buildings)
                beta1_eff = beta0 * loss_NLoS / (d_tx^2);
                gamma_bar1 = beta1_eff * delta_A;
                P_succ_1 = exp(-eta / gamma_bar1); 
            else
                beta1_eff = beta0 * loss_LoS / (d_tx^2);
                gamma_bar1 = beta1_eff * delta_A;
                a1 = sqrt(2 * K_fac);
                b1 = sqrt(2 * (K_fac + 1) * eta / gamma_bar1);
                P_succ_1 = my_marcumq(a1, b1);     
            end
            
            p_out = 1 - (P_succ_1 * P_succ_2);
            max_p = max(max_p, p_out);
        end
        P_outage(i,j) = max_p;
    end
end

% 【极速优化2的秘密武器】：利用数学样条插值，将粗网格平滑升级为 1米x1米 的超高清网格！
[X_fine, Y_fine] = meshgrid(min_x:1:max_x, min_y:1:max_y);
P_outage_smooth = interp2(X_grid, Y_grid, P_outage, X_fine, Y_fine, 'spline');
P_outage_smooth(P_outage_smooth < 0) = 0;
P_outage_smooth(P_outage_smooth > 1) = 1;

figure('Color', 'w', 'Position', [200, 200, 1000, 800]);
% 渲染高清平滑网格
pcolor(X_fine, Y_fine, P_outage_smooth);
shading interp;
colorbar; colormap('jet');
hold on;

for b = 1:num_buildings
    bx = buildings(b,1); by = buildings(b,2); bw = buildings(b,3); bl = buildings(b,4);
    if bx+bw >= min_x && bx <= max_x && by+bl >= min_y && by <= max_y
        patch('XData', [bx bx+bw bx+bw bx], 'YData', [by by by+bl by+bl], ...
              'FaceColor', 'none', 'EdgeColor', 'w', 'LineWidth', 2.0);
        patch('XData', [bx bx+bw bx+bw bx], 'YData', [by by by+bl by+bl], ...
              'FaceColor', 'k', 'FaceAlpha', 0.4, 'EdgeColor', 'none');
    end
end

for k = 1:num_tx_in_cluster
    global_idx_k = tx_global_idx(k);
    pos = tx_positions(k, :);
    curr_r = DT_user_nodes(global_idx_k).error_circle;
    plot(pos(1) + curr_r*cos(theta_circ), pos(2) + curr_r*sin(theta_circ), 'r--', 'LineWidth', 1.2);
    plot(pos(1) + visual_radius*cos(theta_circ), pos(2) + visual_radius*sin(theta_circ), 'b:', 'LineWidth', 1.2);
end

plot(tx_positions(:,1), tx_positions(:,2), 'wo', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(rx_position(1), rx_position(2), 'wp', 'MarkerSize', 14, 'MarkerFaceColor', 'g');

traj_segments = all_trajectories{largest_cluster_idx};
for s = 1:length(traj_segments)
    segment = traj_segments{s};
    if size(segment,1) >= 2
        plot(segment(:,1), segment(:,2), 'w-', 'LineWidth', 2.0);
    end
end

h_err2 = plot(NaN, NaN, 'r--', 'LineWidth', 1.5);
h_vis2 = plot(NaN, NaN, 'b:', 'LineWidth', 1.5);
legend([h_err2, h_vis2], {'Error Circle (Dynamic)', 'Visual Circle (38m)'}, 'Location', 'northeast', 'TextColor', 'w', 'Color', 'none', 'FontSize', 10);

xlabel('X (m)', 'FontSize', 12); ylabel('Y (m)', 'FontSize', 12);
title(sprintf('Hybrid Rayleigh/Rician Outage Heatmap (H=%d m, K=%ddB)', H, K_factor_dB), 'FontSize', 14);
axis equal; xlim([min_x, max_x]); ylim([min_y, max_y]); hold off;
fprintf('仿真完成！\n');

%% ================== 核心功能函数区域 ==================

function draw_3d_building(x, y, w, l, h)
    V = [x, y, 0; x+w, y, 0; x+w, y+l, 0; x, y+l, 0; ...
         x, y, h; x+w, y, h; x+w, y+l, h; x, y+l, h];
    F = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    patch('Vertices', V, 'Faces', F, 'FaceColor', [0.8 0.8 0.8], ...
          'EdgeColor', [0.3 0.3 0.3], 'FaceAlpha', 0.8, 'LineWidth', 0.5);
end

function blocked = is_blocked_by_buildings(point1, point2, buildings)
    blocked = false;
    num_buildings = size(buildings, 1);
    p1 = point1(:); p2 = point2(:); d = p2 - p1;
    if norm(d) < 1e-6, return; end
    
    for b = 1:num_buildings
        bx = buildings(b,1); by = buildings(b,2); bw = buildings(b,3); bl = buildings(b,4); bh = buildings(b,5);
        x_min = bx; x_max = bx + bw; y_min = by; y_max = by + bl;
        
        if p1(3) > bh && p2(3) > bh, continue; end
        
        if abs(d(1)) < 1e-6
            if p1(1) >= x_min && p1(1) <= x_max, t_x_min = 0; t_x_max = 1; else continue; end
        else
            t1 = (x_min - p1(1)) / d(1); t2 = (x_max - p1(1)) / d(1);
            t_x_min = min(t1, t2); t_x_max = max(t1, t2);
        end
        
        if abs(d(2)) < 1e-6
            if p1(2) >= y_min && p1(2) <= y_max, t_y_min = 0; t_y_max = 1; else continue; end
        else
            t3 = (y_min - p1(2)) / d(2); t4 = (y_max - p1(2)) / d(2);
            t_y_min = min(t3, t4); t_y_max = max(t3, t4);
        end
        
        t_low = max([t_x_min, t_y_min, 0]);
        t_high = min([t_x_max, t_y_max, 1]);
        
        if t_low <= t_high + 1e-6 
            t_mid = (t_low + t_high) / 2;
            z_mid = p1(3) + t_mid * d(3);
            if z_mid <= bh, blocked = true; return; end
        end
    end
end

function [best_path, total_distance, total_outage, min_throughput, best_fitness] = ...
    optimize_path_from_current_position(current_pos, return_pos, target_positions, ...
    H, beta0, delta_A, delta_U, eta, v_drone, remaining_time, min_throughput_req, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions)
    
    num_targets = size(target_positions, 1);
    if num_targets == 0
        best_path = []; total_distance = 0; total_outage = 0; min_throughput = inf; best_fitness = 0; return;
    end
    if num_targets == 1
        best_path = 1;
        total_distance = norm(current_pos - target_positions(1,:)) + norm(target_positions(1,:) - return_pos);
        [total_outage, min_throughput] = calculate_path_metrics(current_pos, target_positions, return_pos, best_path, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions);
        best_fitness = evaluate_path_fitness(best_path, current_pos, return_pos, target_positions, H, beta0, delta_A, delta_U, eta, v_drone, remaining_time, min_throughput_req, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions);
        return;
    end
    
    % 【极速优化1】：大幅削减实时重算的遗传算法规模，避免算力黑洞。
    pop_size = min(20, max(10, num_targets * 2));
    max_generations = min(30, max(15, num_targets * 2));
    
    crossover_rate = 0.8; mutation_rate = 0.2; elite_count = max(2, round(pop_size * 0.1));
    population = cell(pop_size, 1);
    for i = 1:pop_size, population{i} = randperm(num_targets); end
    
    fitness = zeros(pop_size, 1);
    for i = 1:pop_size
        fitness(i) = evaluate_path_fitness(population{i}, current_pos, return_pos, target_positions, H, beta0, delta_A, delta_U, eta, v_drone, remaining_time, min_throughput_req, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions);
    end
    
    for gen = 1:max_generations
        [selected_pop, ~] = selection_operation(population, fitness, pop_size);
        offspring_pop = crossover_operation(selected_pop, crossover_rate, num_targets);
        mutated_pop = mutation_operation(offspring_pop, mutation_rate, num_targets);
        new_fitness = zeros(pop_size, 1);
        for i = 1:pop_size
            new_fitness(i) = evaluate_path_fitness(mutated_pop{i}, current_pos, return_pos, target_positions, H, beta0, delta_A, delta_U, eta, v_drone, remaining_time, min_throughput_req, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions);
        end
        [population, fitness] = elitism_operation(population, fitness, mutated_pop, new_fitness, elite_count);
    end
    
    [best_fitness, best_idx] = min(fitness);
    best_path = population{best_idx};
    
    [total_outage, min_throughput] = calculate_path_metrics(current_pos, target_positions, return_pos, best_path, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions);
    total_distance = norm(current_pos - target_positions(best_path(1),:));
    for i = 1:length(best_path)-1
        total_distance = total_distance + norm(target_positions(best_path(i),:) - target_positions(best_path(i+1),:));
    end
    total_distance = total_distance + norm(target_positions(best_path(end),:) - return_pos);
end

function fitness = evaluate_path_fitness(path, start_pos, end_pos, target_pos, ...
    H, beta0, delta_A, delta_U, eta, v_drone, max_time, min_throughput_req, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions)
    
    num_targets = length(path);
    total_distance = norm(start_pos - target_pos(path(1),:));
    for i = 1:num_targets-1
        total_distance = total_distance + norm(target_pos(path(i),:) - target_pos(path(i+1),:));
    end
    total_distance = total_distance + norm(target_pos(path(end),:) - end_pos);
    total_time = total_distance / v_drone;
    
    [total_outage, min_throughput] = calculate_path_metrics(start_pos, target_pos, end_pos, path, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions);
    
    time_penalty = 0; if total_time > max_time, time_penalty = 1000 * (total_time - max_time); end
    throughput_term = 0;
    if min_throughput < min_throughput_req
        throughput_term = 1000 * (min_throughput_req - min_throughput);
    else
        throughput_term = -50 * log(1 + (min_throughput - min_throughput_req));
    end
    
    distance_weight = 0; throughput_weight = 0; outage_weight = 1;

    fitness = distance_weight * (total_distance / 1000) + throughput_weight * throughput_term + outage_weight * (total_outage / 100) + time_penalty;
end

function [total_outage, min_throughput] = calculate_path_metrics(start_pos, target_pos, end_pos, path, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions)
    total_outage = 0; min_throughput = inf; num_targets = length(path);
    [seg_outage, seg_throughput] = calculate_segment_metrics(start_pos, target_pos(path(1),:), all_cluster_positions, end_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
    total_outage = total_outage + seg_outage; min_throughput = min(min_throughput, seg_throughput);
    for i = 1:num_targets-1
        [seg_outage, seg_throughput] = calculate_segment_metrics(target_pos(path(i),:), target_pos(path(i+1),:), all_cluster_positions, end_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
        total_outage = total_outage + seg_outage; min_throughput = min(min_throughput, seg_throughput);
    end
    [seg_outage, seg_throughput] = calculate_segment_metrics(target_pos(path(end),:), end_pos, all_cluster_positions, end_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
    total_outage = total_outage + seg_outage; min_throughput = min(min_throughput, seg_throughput);
end

function [outage_cost, throughput] = calculate_segment_metrics(pos1, pos2, all_targets, station_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac)
    mid_pos = (pos1 + pos2) / 2; max_outage = 0;
    min_throughput = inf; 
    
    pos_UAV = [mid_pos(1), mid_pos(2), H];
    pos_station = [station_pos(1), station_pos(2), 0];
    
    d_rx_dist = max(0.1, norm(pos_UAV - pos_station)); 
    
    if is_blocked_by_buildings(pos_UAV, pos_station, buildings)
        beta2 = beta0 * loss_NLoS / (d_rx_dist^2);
        gamma_bar2 = beta2 * delta_U;
        P_succ_2 = exp(-eta / gamma_bar2); 
    else
        beta2 = beta0 * loss_LoS / (d_rx_dist^2);
        gamma_bar2 = beta2 * delta_U;
        a2 = sqrt(2 * K_fac);
        b2 = sqrt(2 * (K_fac + 1) * eta / gamma_bar2);
        P_succ_2 = my_marcumq(a2, b2);     
    end
    
    if ~isempty(all_targets)
        for i = 1:size(all_targets, 1)
            pos_tx = [all_targets(i,1), all_targets(i,2), 0];
            d_tx_dist = max(0.1, norm(pos_UAV - pos_tx));
            
            if is_blocked_by_buildings(pos_UAV, pos_tx, buildings)
                beta1 = beta0 * loss_NLoS / (d_tx_dist^2);
                gamma_bar1 = beta1 * delta_A;
                P_succ_1 = exp(-eta / gamma_bar1); 
            else
                beta1 = beta0 * loss_LoS / (d_tx_dist^2);
                gamma_bar1 = beta1 * delta_A;
                a1 = sqrt(2 * K_fac);
                b1 = sqrt(2 * (K_fac + 1) * eta / gamma_bar1);
                P_succ_1 = my_marcumq(a1, b1);     
            end
            
            p_out = 1 - (P_succ_1 * P_succ_2);
            max_outage = max(max_outage, p_out);
            
            gamma_eq_bar = min(gamma_bar1, gamma_bar2);
            current_throughput = log2(1 + gamma_eq_bar);
            min_throughput = min(min_throughput, current_throughput);
        end
    else
        min_throughput = log2(1 + gamma_bar2);
    end
    
    outage_cost = max_outage * norm(pos2 - pos1);
    throughput = min_throughput;
end

function q = my_marcumq(a, b)
    if ~isempty(which('marcumq'))
        q = marcumq(a, b);
    elseif ~isempty(which('ncx2cdf'))
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
        [best_fit, best_idx] = min(fitness(candidates));
        selected_pop{i} = population{candidates(best_idx)}; selected_fit(i) = best_fit;
    end
end

function offspring_pop = crossover_operation(parent_pop, crossover_rate, num_genes)
    offspring_pop = parent_pop; num_parents = length(parent_pop);
    for i = 1:2:num_parents-1
        if rand < crossover_rate
            parent1 = parent_pop{i}; parent2 = parent_pop{i+1};
            point1 = randi([1, num_genes-1]); point2 = randi([point1+1, num_genes]);
            child = zeros(1, num_genes); child(point1:point2) = parent1(point1:point2);
            idx = 1;
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
    combined_pop = [old_pop; offspring_pop]; combined_fit = [old_fit; offspring_fit];
    [sorted_fit, sorted_idx] = sort(combined_fit);
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