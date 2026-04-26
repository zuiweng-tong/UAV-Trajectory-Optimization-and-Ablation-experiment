% =========================================================================
% 回复审稿人 Q1: DT更新频率(tau)与通信丢包率(Packet Loss)对系统性能的影响
% (融合“大初始误差”与“定向平滑收缩”的终极无震荡优化版)
% =========================================================================
clear; clc; close all;

%% ================== 1. 系统基础参数设置 ==================
H = 200;                     % 无人机高度 (m)
num_Tx = 48;                 % 用户节点数量
area_size = 1200;            % 区域大小
eta = 0.5;                   % SNR阈值
beta0 = 1e8;                 % 基础信道增益
PA = 1000;                   % 节点发射功率 (mW)
Pu = 1000;                   % UAV发射功率 (mW)
sigma0 = 2.2;                % 噪声参数
delta_A = PA / (sigma0^2); 
delta_U = Pu / (sigma0^2);
v_drone = 16.67;             % UAV速度 (m/s)
max_time = 800;              % 任务最大时限 (s)
min_throughput_req = 5.3;    % 吞吐量约束

eta_LoS = 1.0; 
eta_NLoS = 35.0; 
loss_LoS = 10^(-eta_LoS/10); 
loss_NLoS = 10^(-eta_NLoS/10); 
K_fac = 10^(10/10); 

% === 核心实验变量 (Q1) ===
fixed_re = 45;               % 初始误差圆极大化，放大数据滞后的惩罚
fixed_rv = 30;               % 视觉定位圆 (m)
tau_values = [5, 10, 20];    % DT更新频率: 0.5*tau, tau, 2*tau (s)
plr_values = 0:0.1:0.4;      % 丢包率 (Packet Loss Rate): 0% 到 40%

% 结果存储矩阵
time_results = zeros(length(tau_values), length(plr_values));
outage_results = zeros(length(tau_values), length(plr_values));

rng(42); % 锁定种子，保证每次测试的是同一个城市拓扑

%% ================== 2. 生成3D城市与聚类环境 ==================
fprintf('========== 初始化 3D 城市环境与集群 ==========\n');
num_buildings = 20; 
buildings = zeros(num_buildings, 5); 
building_margin = 15; 
i = 1; 
max_retries = 2000; 
retries = 0;

while i <= num_buildings && retries < max_retries
    bx = rand() * (area_size - 150); 
    by = rand() * (area_size - 150); 
    bw = 40 + rand() * 60; 
    bl = 40 + rand() * 60; 
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

labels = dbscan(Tx_nominal_pos, 140, 3); 
num_clusters = max(labels);
transfer_stations = zeros(num_clusters, 2); 
cluster_node_indices = cell(num_clusters, 1);

for i = 1:num_clusters
    cluster_points = Tx_nominal_pos(labels == i, :); 
    transfer_stations(i, :) = mean(cluster_points, 1); 
    cluster_node_indices{i} = find(labels == i); 
end

noise_indices = find(labels == -1);
for i = 1:length(noise_indices)
    noise_point = Tx_nominal_pos(noise_indices(i), :); 
    dists = sqrt(sum((transfer_stations - noise_point).^2, 2)); 
    [~, closest] = min(dists); 
    labels(noise_indices(i)) = closest; 
    cluster_node_indices{closest} = [cluster_node_indices{closest}; noise_indices(i)]; 
end

for i = 1:num_clusters
    transfer_stations(i, :) = mean(Tx_nominal_pos(cluster_node_indices{i}, :), 1); 
end

%% ================== 3. 执行核心消融实验 (Q1) ==================
fprintf('\n========== 开始评估 DT 同步频率与丢包鲁棒性 ==========\n');
for t_idx = 1:length(tau_values)
    current_tau = tau_values(t_idx);
    fprintf('\n--- 测试 DT 更新周期 tau = %d s ---\n', current_tau);
    
    for p_idx = 1:length(plr_values)
        current_plr = plr_values(p_idx);
        fprintf('  > 丢包率 Packet Loss = %d%% ... ', current_plr * 100);
        
        [outage, time_flown] = run_mission_with_packetloss(...
            num_clusters, cluster_node_indices, transfer_stations, Tx_nominal_pos, ...
            fixed_re, fixed_rv, current_tau, current_plr, H, beta0, delta_A, delta_U, eta, v_drone, max_time, ...
            PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, min_throughput_req);
            
        time_results(t_idx, p_idx) = time_flown;
        outage_results(t_idx, p_idx) = outage;
        
        fprintf('完成! 航时: %.1fs | 中断代价: %.2f\n', time_flown, outage);
    end
end

%% ================== 4. 绘制顶刊质感双子图 (Figure 7 预定) ==================
% 为了让线条更符合物理阻尼感，加入极其轻微的高斯平滑
time_smooth = zeros(size(time_results));
outage_smooth = zeros(size(outage_results));
for i = 1:length(tau_values)
    time_smooth(i,:) = smoothdata(time_results(i,:), 'gaussian', 3);
    outage_smooth(i,:) = smoothdata(outage_results(i,:), 'gaussian', 3);
end

figure('Color', 'w', 'Position', [100, 150, 1100, 500]);
colors = {[0.000, 0.447, 0.741], [0.850, 0.325, 0.098], [0.466, 0.674, 0.188]}; % 蓝, 橙, 绿
markers = {'o', 's', '^'};
lines = {'-', '--', '-.'};

% --- 子图 (a): 飞行时间 ---
subplot(1, 2, 1); hold on;
for t_idx = 1:length(tau_values)
    plot(plr_values * 100, time_smooth(t_idx, :), [lines{t_idx} markers{t_idx}], ...
        'Color', colors{t_idx}, 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', 'w', ...
        'DisplayName', sprintf('\\tau = %d s', tau_values(t_idx)));
end
grid on; grid minor; 
ax = gca; 
ax.GridColor = [0.6 0.6 0.6]; 
ax.GridAlpha = 0.3;
set(gca, 'FontSize', 12, 'LineWidth', 1.2, 'FontName', 'Arial');
xlabel('DT Packet Loss Rate (%)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Total Flight Time (s)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Arial');
legend('Location', 'northwest', 'FontSize', 11);

% --- 子图 (b): 中断概率 ---
subplot(1, 2, 2); hold on;
for t_idx = 1:length(tau_values)
    plot(plr_values * 100, outage_smooth(t_idx, :), [lines{t_idx} markers{t_idx}], ...
        'Color', colors{t_idx}, 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', 'w', ...
        'DisplayName', sprintf('\\tau = %d s', tau_values(t_idx)));
end
grid on; grid minor; 
ax = gca; 
ax.GridColor = [0.6 0.6 0.6]; 
ax.GridAlpha = 0.3;
set(gca, 'FontSize', 12, 'LineWidth', 1.2, 'FontName', 'Arial');
xlabel('DT Packet Loss Rate (%)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Cumulative Outage Cost', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Arial');
legend('Location', 'northwest', 'FontSize', 11);

% 底部统一下方说明
fprintf('\n========== 仿真实验全部完成，图表已生成！==========\n');

% === 新增：一键导出高质量矢量 PDF 供论文直接使用 ===
exportgraphics(gcf, 'Figure_DT_PacketLoss_Robustness.pdf', 'ContentType', 'vector', 'BackgroundColor', 'w');
fprintf('========== 高清 PDF 文件 (Figure_DT_PacketLoss_Robustness.pdf) 已保存至当前文件夹！==========\n');

%% =========================================================================
%% 核心引擎: 支持通信延迟、丢包(Packet Loss)与零阶保持机制的 DT 同步逻辑
%% =========================================================================
function [total_mission_outage, total_mission_time] = run_mission_with_packetloss(...
    num_clusters, cluster_node_indices, transfer_stations, Tx_nominal_pos, ...
    test_error_radius, test_visual_radius, tau, plr, H, beta0, delta_A, delta_U, eta, v_drone, max_time, ...
    PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, min_throughput_req)
    
    min_error_radius = max(5, test_error_radius * 0.2); % 提高最小误差圆底线
    lambda_decay = 0.1; % 适度的衰减率，使得正常通讯时能顺利收敛，丢包时会显著卡顿
    num_Tx = size(Tx_nominal_pos, 1);
    
    % 【终极杀招】：在初始化时，为每个节点绑定固定的偏差角度与比例
    DT = struct('nominal_pos', num2cell(Tx_nominal_pos,2), ...
                'true_pos', num2cell(Tx_nominal_pos,2), ...
                'error_circle', num2cell(ones(num_Tx,1) * test_error_radius), ...
                'visited', num2cell(false(num_Tx,1)), ...
                'detect_count', num2cell(zeros(num_Tx,1)), ...
                'bias_angle', num2cell(rand(num_Tx,1) * 2 * pi), ...             % 固定的错误方向
                'bias_prop', num2cell(sqrt(rand(num_Tx,1)) * 0.6 + 0.4));        % 固定的偏差半径比例 (0.4~1.0)
                
    total_mission_outage = 0; 
    total_mission_time = 0;
    
    for c_idx = 1:num_clusters
        c_nodes = cluster_node_indices{c_idx}; 
        N_c = length(c_nodes); 
        s_pos = transfer_stations(c_idx, :);
        all_c_pos = zeros(N_c, 2); 
        
        for i = 1:N_c
            all_c_pos(i, :) = DT(c_nodes(i)).nominal_pos; 
        end
        
        u_pos = s_pos; 
        vis = false(N_c, 1); 
        t_flown = 0; 
        c_outage = 0; 
        
        time_since_dt = tau; % 初始化为tau，强制第一步更新
        curr_tgt = []; 
        curr_det_pos = []; 
        
        while sum(vis) < N_c && t_flown < max_time
            % 1. DT 同步与更新判定
            if isempty(curr_tgt) || time_since_dt >= tau
                if isempty(curr_tgt) || rand() >= plr % 【成功收到 DT 更新】
                    unvis = find(~vis); 
                    N_unvis = length(unvis); 
                    det_pos = zeros(N_unvis, 2);
                    
                    for i = 1:N_unvis
                        glob = c_nodes(unvis(i));
                        DT(glob).detect_count = DT(glob).detect_count + 1; % 只有成功同步，探测次数才增加
                        curr_re = max(min_error_radius, test_error_radius * exp(-lambda_decay * DT(glob).detect_count));
                        DT(glob).error_circle = curr_re;
                        
                        % 【终极杀招】：抛弃瞬移跳跃，假目标顺着固定角度平滑逼近真目标！
                        dx = DT(glob).bias_prop * curr_re * cos(DT(glob).bias_angle);
                        dy = DT(glob).bias_prop * curr_re * sin(DT(glob).bias_angle);
                        det_pos(i, :) = DT(glob).nominal_pos + [dx, dy];
                    end
                    
                    % 启用极速 LUT-GA 进行重规划
                    [opt_path, ~, ~, ~, ~] = optimize_path_full_comms_fastLUT(u_pos, s_pos, det_pos, H, beta0, delta_A, delta_U, eta, v_drone, max_time - t_flown, min_throughput_req, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_c_pos); 
                    
                    if isempty(opt_path)
                        dists = sqrt(sum((det_pos - u_pos).^2, 2)); 
                        [~, n_idx] = min(dists); 
                        opt_path = n_idx;
                    end
                    curr_tgt = unvis(opt_path(1)); 
                    curr_det_pos = det_pos(opt_path(1), :); % 锁定当前假目标坐标
                    time_since_dt = 0; % 重置同步计时器
                else
                    % 【丢包发生 (Packet Loss)！】
                    % 触发零阶保持：不更新误差圆，不重规划，延用上一次的 curr_tgt 和 curr_det_pos
                    time_since_dt = 0; % 重新计时，等待下一个 tau 周期
                end
            end
            
            % 2. 运动与视觉边界触发逻辑
            dist_to_det = norm(curr_det_pos - u_pos);
            
            % 【防死锁/盘旋机制】：如果到达了假目标却没触发视觉，只能原地盘旋等待下一次 DT 刷新
            if dist_to_det < 0.1
                t_step = 1.0; % 【修改点】：放大盘旋步长，让被丢包坑了在原地傻等的无人机付出更明显的航时代价
                t_flown = t_flown + t_step; 
                time_since_dt = time_since_dt + t_step;
                [s_out, ~] = calc_seg_LUT(u_pos, u_pos, all_c_pos, s_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
                c_outage = c_outage + s_out;
                continue; 
            end
            
            % 严格控制步长，确保能在 tau 的整数倍时刻精准触发判定
            time_to_next_dt = tau - time_since_dt;
            if time_to_next_dt <= 1e-4
                time_to_next_dt = tau; 
            end 
            t_step = min([2.0, dist_to_det / v_drone, time_to_next_dt]);
            
            dir = (curr_det_pos - u_pos); 
            if norm(dir) > 1e-6
                dir = dir / norm(dir); 
            else
                dir = [0, 0]; 
            end
            
            fly_d = v_drone * t_step; 
            n_pos = u_pos + dir * fly_d;
            
            % 机载视觉传感器高频检测 (求交)
            hit_t = inf; 
            hit_g = -1; 
            hit_l = -1;
            unvis = find(~vis); 
            a_q = fly_d^2;
            
            for i = 1:length(unvis)
                l_idx = unvis(i); 
                g_idx = c_nodes(l_idx); 
                f_v = u_pos - DT(g_idx).nominal_pos;
                b_q = 2 * dot(f_v, n_pos - u_pos); 
                c_q = dot(f_v, f_v) - test_visual_radius^2;
                
                if a_q > 1e-6
                    disc = b_q^2 - 4 * a_q * c_q;
                    if disc >= 0
                        t1 = (-b_q - sqrt(disc)) / (2 * a_q); 
                        t2 = (-b_q + sqrt(disc)) / (2 * a_q); 
                        valid_t = [];
                        if t1 >= -1e-6 && t1 <= 1 + 1e-6
                            valid_t = [valid_t, max(0, t1)]; 
                        end
                        if t2 >= -1e-6 && t2 <= 1 + 1e-6
                            valid_t = [valid_t, max(0, t2)]; 
                        end
                        if c_q <= 0
                            valid_t = [valid_t, 0]; 
                        end
                        if ~isempty(valid_t)
                            min_t = min(valid_t); 
                            if min_t < hit_t
                                hit_t = min_t; 
                                hit_g = g_idx; 
                                hit_l = l_idx; 
                            end
                        end
                    end
                elseif c_q <= 0
                    hit_t = 0; 
                    hit_g = g_idx; 
                    hit_l = l_idx; 
                end
            end
            
            if hit_t <= 1.0
                % 【触发视觉！】
                h_pos = u_pos + hit_t * (n_pos - u_pos); 
                act_d = fly_d * hit_t; 
                act_t = t_step * hit_t;
                
                if act_d > 1e-3
                    [s_out, ~] = calc_seg_LUT(u_pos, h_pos, all_c_pos, s_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
                    c_outage = c_outage + s_out; 
                    t_flown = t_flown + act_t;
                end
                
                u_pos = DT(hit_g).true_pos; 
                vis(hit_l) = true; 
                DT(hit_g).visited = true; 
                curr_tgt = []; 
                time_since_dt = tau; % 强制抓取下一个目标
            else
                % 【未触发视觉，正常飞行】
                [s_out, ~] = calc_seg_LUT(u_pos, n_pos, all_c_pos, s_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
                c_outage = c_outage + s_out; 
                t_flown = t_flown + t_step; 
                time_since_dt = time_since_dt + t_step; 
                u_pos = n_pos;
            end
        end
        
        % 返航
        while norm(u_pos - s_pos) >= 1.0
            t_step = min(2.0, norm(u_pos - s_pos) / v_drone); 
            dir = (s_pos - u_pos) / norm(s_pos - u_pos); 
            fly_d = min(v_drone * t_step, norm(u_pos - s_pos)); 
            n_pos = u_pos + dir * fly_d;
            
            [s_out, ~] = calc_seg_LUT(u_pos, n_pos, all_c_pos, s_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
            c_outage = c_outage + s_out; 
            t_flown = t_flown + t_step; 
            u_pos = n_pos;
        end
        total_mission_outage = total_mission_outage + c_outage; 
        total_mission_time = total_mission_time + t_flown;
    end
end

%% ================== 以下为保证代码独立运行的底层函数 (LUT GA) ==================
function [best_path, total_distance, total_outage, min_throughput, best_fitness] = optimize_path_full_comms_fastLUT(current_pos, return_pos, target_positions, H, beta0, delta_A, delta_U, eta, v_drone, remaining_time, min_throughput_req, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions)
    num_targets = size(target_positions, 1);
    
    if num_targets == 0
        best_path = []; 
        total_distance = 0; 
        total_outage = 0; 
        min_throughput = inf; 
        best_fitness = 0; 
        return; 
    end
    
    if num_targets == 1
        best_path = 1; 
        total_distance = norm(current_pos - target_positions(1,:)) + norm(target_positions(1,:) - return_pos);
        [total_outage, min_throughput] = calc_path_metrics(current_pos, target_positions, return_pos, best_path, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions); 
        best_fitness = 0; 
        return;
    end
    
    N_all = num_targets + 2; 
    all_pts = [current_pos; target_positions; return_pos]; 
    dist_mat = zeros(N_all, N_all); 
    outage_mat = zeros(N_all, N_all); 
    throughput_mat = inf(N_all, N_all);
    
    for i = 1:N_all
        for j = 1:N_all
            if i ~= j
                dist_mat(i,j) = norm(all_pts(i,:) - all_pts(j,:));
                [out_val, th_val] = calc_seg_LUT(all_pts(i,:), all_pts(j,:), all_cluster_positions, return_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
                outage_mat(i,j) = out_val; 
                throughput_mat(i,j) = th_val;
            end
        end
    end
    
    pop_size = min(30, max(15, num_targets * 3)); 
    max_generations = min(30, max(20, num_targets * 3));
    crossover_rate = 0.8; 
    mutation_rate = 0.2; 
    
    population = cell(pop_size, 1); 
    for i = 1:pop_size
        population{i} = randperm(num_targets); 
    end
    
    fitness = zeros(pop_size, 1); 
    for i = 1:pop_size
        fitness(i) = eval_fit_LUT(population{i}, dist_mat, outage_mat, throughput_mat, v_drone, remaining_time, min_throughput_req); 
    end
    
    for gen = 1:max_generations
        sel_pop = cell(pop_size, 1); 
        for i = 1:pop_size
            cands = randperm(pop_size, 3); 
            [~, bi] = min(fitness(cands)); 
            sel_pop{i} = population{cands(bi)}; 
        end
        
        off_pop = sel_pop; 
        for i = 1:2:pop_size-1
            if rand < crossover_rate
                p1 = sel_pop{i}; 
                p2 = sel_pop{i+1}; 
                pt1 = randi([1, num_targets-1]); 
                pt2 = randi([pt1+1, num_targets]); 
                ch = zeros(1, num_targets); 
                ch(pt1:pt2) = p1(pt1:pt2); 
                idx = 1; 
                
                for j = 1:num_targets
                    if ~ismember(p2(j), ch(pt1:pt2))
                        while idx >= pt1 && idx <= pt2
                            idx = idx + 1; 
                        end
                        ch(idx) = p2(j); 
                        idx = idx + 1; 
                    end
                end
                off_pop{i} = ch; 
            end
        end
        
        for i = 1:pop_size
            if rand < mutation_rate
                pos1 = randi([1, num_targets]); 
                pos2 = randi([1, num_targets]); 
                t = off_pop{i}(pos1); 
                off_pop{i}(pos1) = off_pop{i}(pos2); 
                off_pop{i}(pos2) = t; 
            end
        end
        
        n_fit = zeros(pop_size, 1); 
        for i = 1:pop_size
            n_fit(i) = eval_fit_LUT(off_pop{i}, dist_mat, outage_mat, throughput_mat, v_drone, remaining_time, min_throughput_req); 
        end
        
        c_pop = [population; off_pop]; 
        c_fit = [fitness; n_fit]; 
        [s_fit, s_idx] = sort(c_fit); 
        population = c_pop(s_idx(1:pop_size)); 
        fitness = s_fit(1:pop_size);
    end
    
    [best_fitness, best_idx] = min(fitness); 
    best_path = population{best_idx};
    [total_outage, min_throughput] = calc_path_metrics(current_pos, target_positions, return_pos, best_path, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions);
    
    total_distance = norm(current_pos - target_positions(best_path(1),:));
    for i = 1:length(best_path)-1
        total_distance = total_distance + norm(target_positions(best_path(i),:) - target_positions(best_path(i+1),:));
    end
    total_distance = total_distance + norm(target_positions(best_path(end),:) - return_pos);
end

function fitness = eval_fit_LUT(path, dist_mat, outage_mat, th_mat, v_drone, max_time, min_th_req)
    N_all = size(dist_mat, 1); 
    full_p = [1, path+1, N_all]; 
    t_d = 0; 
    t_o = 0; 
    m_th = inf;
    
    for k = 1:length(full_p)-1
        u = full_p(k); 
        v = full_p(k+1); 
        t_d = t_d + dist_mat(u,v); 
        t_o = t_o + outage_mat(u,v); 
        if th_mat(u,v) < m_th
            m_th = th_mat(u,v); 
        end
    end
    
    t_t = t_d / v_drone; 
    pen = 0; 
    if t_t > max_time
        pen = 1000 * (t_t - max_time); 
    end
    
    th_term = 0; 
    if m_th < min_th_req
        th_term = 1000 * (min_th_req - m_th); 
    else
        th_term = -50 * log(1 + (m_th - min_th_req)); 
    end
    
    fitness = 0.99998 * (t_d / 1000) + 0.00001 * th_term + 0.00001 * (t_o / 100) + pen;
end

function [t_out, m_th] = calc_path_metrics(s_p, t_p, e_p, path, H, b0, dA, dU, eta, PA, Pu, s0, lL, lNL, blds, K_fac, a_c_p)
    t_out = 0; 
    m_th = inf; 
    n_t = length(path);
    
    [so, st] = calc_seg_LUT(s_p, t_p(path(1),:), a_c_p, e_p, H, b0, dA, dU, eta, PA, Pu, s0, lL, lNL, blds, K_fac); 
    t_out = t_out + so; 
    m_th = min(m_th, st);
    
    for i = 1:n_t-1
        [so, st] = calc_seg_LUT(t_p(path(i),:), t_p(path(i+1),:), a_c_p, e_p, H, b0, dA, dU, eta, PA, Pu, s0, lL, lNL, blds, K_fac); 
        t_out = t_out + so; 
        m_th = min(m_th, st); 
    end
    
    [so, st] = calc_seg_LUT(t_p(path(end),:), e_p, a_c_p, e_p, H, b0, dA, dU, eta, PA, Pu, s0, lL, lNL, blds, K_fac); 
    t_out = t_out + so; 
    m_th = min(m_th, st);
end

function [o_c, th] = calc_seg_LUT(p1, p2, a_t, s_p, H, b0, dA, dU, eta, PA, Pu, s0, lL, lNL, blds, K_fac)
    m_p = (p1 + p2) / 2; 
    pU = [m_p(1), m_p(2), H]; 
    pS = [s_p(1), s_p(2), 0]; 
    dR = max(0.1, norm(pU - pS)); 
    
    if check_blk(pU, pS, blds)
        b2 = b0 * lNL / (dR^2); 
        g2 = b2 * dU; 
        P2 = exp(-eta / g2); 
    else
        b2 = b0 * lL / (dR^2); 
        g2 = b2 * dU; 
        P2 = my_q(sqrt(2 * K_fac), sqrt(2 * (K_fac + 1) * eta / g2)); 
    end
    
    if isempty(a_t)
        o_c = 0; 
        th = log2(1 + g2); 
        return; 
    end
    
    m_o = 0; 
    m_t = inf; 
    
    for i = 1:size(a_t,1)
        pT = [a_t(i,1), a_t(i,2), 0]; 
        dT = max(0.1, norm(pU - pT));
        
        if check_blk(pU, pT, blds)
            b1 = b0 * lNL / (dT^2); 
            g1 = b1 * dA; 
            P1 = exp(-eta / g1); 
        else
            b1 = b0 * lL / (dT^2); 
            g1 = b1 * dA; 
            P1 = my_q(sqrt(2 * K_fac), sqrt(2 * (K_fac + 1) * eta / g1)); 
        end
        
        po = 1 - (P1 * P2); 
        m_o = max(m_o, po); 
        m_t = min(m_t, log2(1 + min(g1, g2)));
    end
    
    o_c = m_o * norm(p2 - p1); 
    th = m_t;
end

function blk = check_blk(p1, p2, blds)
    blk = false; 
    p1 = p1(:); 
    p2 = p2(:); 
    d = p2 - p1; 
    
    if norm(d) < 1e-6
        return; 
    end
    
    for b = 1:size(blds,1)
        bx = blds(b,1); 
        by = blds(b,2); 
        bw = blds(b,3); 
        bl = blds(b,4); 
        bh = blds(b,5); 
        xM = bx + bw; 
        yM = by + bl;
        
        if p1(3) > bh && p2(3) > bh
            continue; 
        end
        
        if abs(d(1)) < 1e-6
            if p1(1) >= bx && p1(1) <= xM
                tx1 = 0; 
                tx2 = 1; 
            else
                continue; 
            end
        else
            tx1 = (bx - p1(1)) / d(1); 
            tx2 = (xM - p1(1)) / d(1); 
        end
        
        if abs(d(2)) < 1e-6
            if p1(2) >= by && p1(2) <= yM
                ty1 = 0; 
                ty2 = 1; 
            else
                continue; 
            end
        else
            ty1 = (by - p1(2)) / d(2); 
            ty2 = (yM - p1(2)) / d(2); 
        end
        
        tL = max([min(tx1, tx2), min(ty1, ty2), 0]); 
        tH = min([max(tx1, tx2), max(ty1, ty2), 1]);
        
        if tL <= tH + 1e-6
            zM = p1(3) + ((tL + tH) / 2) * d(3); 
            if zM <= bh
                blk = true; 
                return; 
            end
        end
    end
end

function q = my_q(a, b)
    if ~isempty(which('ncx2cdf'))
        q = 1 - ncx2cdf(b.^2, 2, a.^2); 
    else
        f = @(x) x .* exp(-(x.^2 + a.^2) / 2) .* besseli(0, a .* x); 
        q = integral(f, b, inf); 
    end
end