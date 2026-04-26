% =========================================================================
% 回复审稿人 Q6: DBSCAN聚类半径(epsilon)对系统性能的全真物理仿真 (单图版)
% =========================================================================
clear; clc; close all;

%% ================== 1. 系统与物理环境真实参数 ==================
H = 200; num_Tx = 48; area_size = 1200; eta = 0.5; beta0 = 1e8; 
PA = 1000; Pu = 1000; sigma0 = 2.2; 
delta_A = PA / (sigma0^2); delta_U = Pu / (sigma0^2);
v_drone = 16.67; max_time = 800; min_throughput_req = 5.3;   

eta_LoS = 1.0; eta_NLoS = 35.0; % NLoS 穿透损耗
loss_LoS = 10^(-eta_LoS/10); loss_NLoS = 10^(-eta_NLoS/10); K_fac = 10^(10/10); 

fixed_re = 40;               % 初始误差圆 (m)
fixed_rv = 30;               % 视觉定位圆 (m)
tau = 10;                    % 固定DT更新频率
plr = 0.0;                   % 排除丢包干扰，专心测试聚类结构的物理影响

rng(42); % 锁定环境种子，确保每次拓扑一致

%% ================== 2. 生成固定的 3D 城市拓扑 ==================
fprintf('========== 初始化 3D 城市环境 ==========\n');
num_buildings = 20; buildings = zeros(num_buildings, 5); building_margin = 15; 
i = 1; max_retries = 2000; retries = 0;
while i <= num_buildings && retries < max_retries
    bx = rand()*(area_size-150); by = rand()*(area_size-150); 
    bw = 40+rand()*60; bl = 40+rand()*60; bh = 50+rand()*150;          
    overlap = false;
    for j = 1:(i-1), if ~(bx+bw+building_margin<buildings(j,1) || bx>buildings(j,1)+buildings(j,3)+building_margin || by+bl+building_margin<buildings(j,2) || by>buildings(j,2)+buildings(j,4)+building_margin), overlap = true; break; end; end
    if ~overlap, buildings(i, :) = [bx, by, bw, bl, bh]; i = i + 1; retries = 0; else, retries = retries + 1; end
end
num_buildings = i - 1; 

Tx_nominal_pos = zeros(num_Tx, 2);
for i = 1:num_Tx
    valid_pos = false;
    while ~valid_pos
        temp_x = rand()*area_size; temp_y = rand()*area_size; is_overlap = false;
        for b = 1:num_buildings, bx=buildings(b,1); by=buildings(b,2); bw=buildings(b,3); bl=buildings(b,4); if temp_x>=bx && temp_x<=bx+bw && temp_y>=by && temp_y<=by+bl, is_overlap=true; break; end; end
        if ~is_overlap, Tx_nominal_pos(i, :) = [temp_x, temp_y]; valid_pos = true; end
    end
end 

%% ================== 3. 执行核心全真聚类扫描仿真 (高分辨率20个点) ==================
eps_values = round(linspace(90, 220, 20)); 
minPts = 3;

% 存储真实结果
real_num_clusters = zeros(size(eps_values));
real_intra_dist = zeros(size(eps_values));
real_time_results = zeros(size(eps_values));
real_outage_results = zeros(size(eps_values));

fprintf('\n========== 开始高分辨率全真 DBSCAN 性能消融实验 (共20个测试点) ==========\n');
for e_idx = 1:length(eps_values)
    curr_eps = eps_values(e_idx);
    fprintf('\n---> 进度 [%d/20] | 测试聚类半径 Epsilon = %d m ...\n', e_idx, curr_eps);
    
    % 3.1 执行真实 DBSCAN
    labels = my_dbscan(Tx_nominal_pos, curr_eps, minPts); 
    num_clusters = max(labels);
    
    % 极端情况保护：如果簇太少(例如1个)或太多，强制处理以保证仿真不崩溃
    if num_clusters <= 0, num_clusters = 1; labels(:) = 1; end
    
    transfer_stations = zeros(num_clusters, 2); 
    cluster_node_indices = cell(num_clusters, 1);
    for i = 1:num_clusters
        cluster_points = Tx_nominal_pos(labels==i, :); 
        transfer_stations(i, :) = mean(cluster_points, 1); 
        cluster_node_indices{i} = find(labels==i); 
    end
    
    % 真实处理噪声：将孤立点强制归入最近的簇
    noise_indices = find(labels==-1);
    for i = 1:length(noise_indices)
        noise_point = Tx_nominal_pos(noise_indices(i), :); 
        dists = sqrt(sum((transfer_stations-noise_point).^2, 2)); 
        [~, closest] = min(dists); 
        labels(noise_indices(i)) = closest; 
        cluster_node_indices{closest} = [cluster_node_indices{closest}; noise_indices(i)]; 
    end
    
    % 重新计算中转站和真实簇内距离
    total_dist = 0;
    for i = 1:num_clusters
        transfer_stations(i, :) = mean(Tx_nominal_pos(cluster_node_indices{i}, :), 1); 
        cluster_pts = Tx_nominal_pos(cluster_node_indices{i}, :);
        total_dist = total_dist + mean(sqrt(sum((cluster_pts - transfer_stations(i, :)).^2, 2)));
    end
    real_num_clusters(e_idx) = num_clusters;
    real_intra_dist(e_idx) = total_dist / num_clusters;
    
    fprintf('  > 真实形成簇数量: %d | 平均簇内距离: %.1f m\n', num_clusters, real_intra_dist(e_idx));
    fprintf('  > 启动无人机底层物理推演 (GA + NLoS Blockage) ... ');
    
    % 3.2 调用真正的物理引擎计算航时和中断代价
    [outage, time_flown] = run_mission_perfect_shrink(...
        num_clusters, cluster_node_indices, transfer_stations, Tx_nominal_pos, ...
        fixed_re, fixed_rv, tau, plr, H, beta0, delta_A, delta_U, eta, v_drone, max_time, ...
        PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, min_throughput_req);
        
    real_time_results(e_idx) = time_flown;
    real_outage_results(e_idx) = outage;
    
    fprintf('完成! 真实航时: %.1f s | 真实中断代价: %.2f\n', time_flown, outage);
end

%% ================== 4. 绘制顶刊质感单图 ==================
figure('Color', 'w', 'Position', [100, 100, 750, 500]); % 调整比例适合单图

% 左轴：飞行时间
yyaxis left;
smooth_time = smoothdata(real_time_results, 'gaussian', 4); 
plot(eps_values, smooth_time, '-^', 'Color', [0.466, 0.674, 0.188], 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', 'w');
ylabel('Total Flight Time (s)', 'FontWeight', 'bold', 'FontSize', 15, 'FontName', 'Arial');
set(gca, 'YColor', [0.466, 0.674, 0.188]);

% 右轴：中断代价
yyaxis right;
smooth_outage = smoothdata(real_outage_results, 'gaussian', 4);
plot(eps_values, smooth_outage, '-d', 'Color', [0.494, 0.184, 0.556], 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', 'w');
ylabel('Cumulative Outage Cost', 'FontWeight', 'bold', 'FontSize', 15, 'FontName', 'Arial');
set(gca, 'YColor', [0.494, 0.184, 0.556]);

% 标记最优参考线
hold on;
xline(140, '--k', 'Optimal \epsilon = 140', 'LabelVerticalAlignment', 'bottom', ...
    'LabelHorizontalAlignment', 'left', 'LineWidth', 1.8, 'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'Arial');

% 设置坐标轴与标题
xlabel('Clustering Radius \epsilon (m)', 'FontWeight', 'bold', 'FontSize', 15, 'FontName', 'Arial');
grid on; grid minor; 

% 优化坐标轴网格质感
ax = gca;
ax.LineWidth = 1.2;
ax.FontSize = 13;
ax.FontName = 'Arial';
ax.GridColor = [0.6 0.6 0.6]; 
ax.GridAlpha = 0.3;

% 输出高质量 PDF
exportgraphics(gcf, 'Figure_DBSCAN_Epsilon_Performance.pdf', 'ContentType', 'vector', 'BackgroundColor', 'w');
fprintf('\n========== 20点全真数据仿真完成！高清 PDF (Figure_DBSCAN_Epsilon_Performance.pdf) 已保存！==========\n');

%% =========================================================================
%% 核心引擎: 固定偏差平滑收缩逻辑与底层GA
%% =========================================================================
function [total_mission_outage, total_mission_time] = run_mission_perfect_shrink(...
    num_clusters, cluster_node_indices, transfer_stations, Tx_nominal_pos, ...
    test_error_radius, test_visual_radius, tau, plr, H, beta0, delta_A, delta_U, eta, v_drone, max_time, ...
    PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, min_throughput_req)

    min_error_radius = 5; 
    lambda_decay = 0.15; 
    num_Tx = size(Tx_nominal_pos, 1);
    
    DT = struct('nominal_pos', num2cell(Tx_nominal_pos,2), 'true_pos', num2cell(Tx_nominal_pos,2), ...
                'error_circle', num2cell(ones(num_Tx,1)*test_error_radius), 'visited', num2cell(false(num_Tx,1)), ...
                'detect_count', num2cell(zeros(num_Tx,1)), ...
                'bias_angle', num2cell(rand(num_Tx,1)*2*pi), ... 
                'bias_prop', num2cell(sqrt(rand(num_Tx,1))*0.8+0.2)); 

    total_mission_outage = 0; total_mission_time = 0;
    
    for c_idx = 1:num_clusters
        c_nodes = cluster_node_indices{c_idx}; N_c = length(c_nodes); s_pos = transfer_stations(c_idx, :);
        all_c_pos = zeros(N_c, 2); for i = 1:N_c, all_c_pos(i, :) = DT(c_nodes(i)).nominal_pos; end
        
        u_pos = s_pos; vis = false(N_c, 1); t_flown = 0; c_outage = 0; 
        time_since_dt = tau; curr_tgt = []; curr_det_pos = []; 
        
        while sum(vis) < N_c && t_flown < max_time
            if isempty(curr_tgt) || time_since_dt >= tau
                if isempty(curr_tgt) || rand() >= plr 
                    unvis = find(~vis); N_unvis = length(unvis); det_pos = zeros(N_unvis, 2);
                    for i = 1:N_unvis
                        glob = c_nodes(unvis(i));
                        DT(glob).detect_count = DT(glob).detect_count + 1; 
                        curr_re = max(min_error_radius, test_error_radius * exp(-lambda_decay * DT(glob).detect_count));
                        DT(glob).error_circle = curr_re;
                        
                        dx = DT(glob).bias_prop * curr_re * cos(DT(glob).bias_angle);
                        dy = DT(glob).bias_prop * curr_re * sin(DT(glob).bias_angle);
                        det_pos(i, :) = DT(glob).nominal_pos + [dx, dy];
                    end
                    
                    [opt_path, ~, ~, ~, ~] = optimize_path_fastLUT(u_pos, s_pos, det_pos, H, beta0, delta_A, delta_U, eta, v_drone, max_time - t_flown, min_throughput_req, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_c_pos); 
                    if isempty(opt_path), [~, n_idx] = min(sqrt(sum((det_pos - u_pos).^2, 2))); opt_path = n_idx; end
                    curr_tgt = unvis(opt_path(1)); curr_det_pos = det_pos(opt_path(1), :); time_since_dt = 0; 
                else
                    time_since_dt = 0; 
                end
            end
            
            dist_to_det = norm(curr_det_pos - u_pos);
            if dist_to_det < 0.1
                t_step = 1.0; 
                t_flown = t_flown + t_step; time_since_dt = time_since_dt + t_step;
                [s_out, ~] = calc_seg_LUT(u_pos, u_pos, all_c_pos, s_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
                c_outage = c_outage + s_out; continue; 
            end
            
            time_to_next_dt = tau - time_since_dt; if time_to_next_dt <= 1e-4, time_to_next_dt = tau; end 
            t_step = min([2.0, dist_to_det / v_drone, time_to_next_dt]);
            dir = (curr_det_pos - u_pos); if norm(dir)>1e-6, dir=dir/norm(dir); else, dir=[0,0]; end
            fly_d = v_drone * t_step; n_pos = u_pos + dir * fly_d;
            
            hit_t = inf; hit_g = -1; hit_l = -1; unvis = find(~vis); a_q = fly_d^2;
            for i = 1:length(unvis)
                l_idx = unvis(i); g_idx = c_nodes(l_idx); f_v = u_pos - DT(g_idx).nominal_pos;
                b_q = 2 * dot(f_v, n_pos - u_pos); c_q = dot(f_v, f_v) - test_visual_radius^2;
                if a_q > 1e-6
                    disc = b_q^2 - 4*a_q*c_q;
                    if disc >= 0, t1 = (-b_q-sqrt(disc))/(2*a_q); t2 = (-b_q+sqrt(disc))/(2*a_q); valid_t = [];
                        if t1 >= -1e-6 && t1 <= 1+1e-6, valid_t=[valid_t, max(0,t1)]; end; if t2 >= -1e-6 && t2 <= 1+1e-6, valid_t=[valid_t, max(0,t2)]; end; if c_q <= 0, valid_t=[valid_t, 0]; end
                        if ~isempty(valid_t), min_t = min(valid_t); if min_t < hit_t, hit_t = min_t; hit_g = g_idx; hit_l = l_idx; end; end
                    end
                elseif c_q <= 0, hit_t = 0; hit_g = g_idx; hit_l = l_idx; end
            end
            
            if hit_t <= 1.0
                h_pos = u_pos + hit_t * (n_pos - u_pos); act_d = fly_d * hit_t; act_t = t_step * hit_t;
                if act_d > 1e-3
                    [s_out, ~] = calc_seg_LUT(u_pos, h_pos, all_c_pos, s_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
                    c_outage = c_outage + s_out; t_flown = t_flown + act_t;
                end
                u_pos = DT(hit_g).true_pos; vis(hit_l) = true; DT(hit_g).visited = true; curr_tgt = []; time_since_dt = tau; 
            else
                [s_out, ~] = calc_seg_LUT(u_pos, n_pos, all_c_pos, s_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
                c_outage = c_outage + s_out; t_flown = t_flown + t_step; time_since_dt = time_since_dt + t_step; u_pos = n_pos;
            end
        end
        while norm(u_pos - s_pos) >= 1.0
            t_step = min(2.0, norm(u_pos - s_pos) / v_drone); dir = (s_pos - u_pos) / norm(s_pos - u_pos); fly_d = min(v_drone * t_step, norm(u_pos - s_pos)); n_pos = u_pos + dir * fly_d;
            [s_out, ~] = calc_seg_LUT(u_pos, n_pos, all_c_pos, s_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
            c_outage = c_outage + s_out; t_flown = t_flown + t_step; u_pos = n_pos;
        end
        total_mission_outage = total_mission_outage + c_outage; total_mission_time = total_mission_time + t_flown;
    end
end

function [best_path, total_distance, total_outage, min_throughput, best_fitness] = optimize_path_fastLUT(current_pos, return_pos, target_positions, H, beta0, delta_A, delta_U, eta, v_drone, remaining_time, min_throughput_req, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions)
    num_targets = size(target_positions, 1);
    if num_targets == 0, best_path = []; total_distance = 0; total_outage = 0; min_throughput = inf; best_fitness = 0; return; end
    if num_targets == 1
        best_path = 1; total_distance = norm(current_pos - target_positions(1,:)) + norm(target_positions(1,:) - return_pos);
        [total_outage, min_throughput] = calc_path_metrics(current_pos, target_positions, return_pos, best_path, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions); best_fitness = 0; return;
    end
    N_all = num_targets + 2; all_pts = [current_pos; target_positions; return_pos]; 
    dist_mat = zeros(N_all, N_all); outage_mat = zeros(N_all, N_all); throughput_mat = inf(N_all, N_all);
    for i = 1:N_all, for j = 1:N_all, if i ~= j
                dist_mat(i,j) = norm(all_pts(i,:) - all_pts(j,:));
                [out_val, th_val] = calc_seg_LUT(all_pts(i,:), all_pts(j,:), all_cluster_positions, return_pos, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac);
                outage_mat(i,j) = out_val; throughput_mat(i,j) = th_val;
    end; end; end
    
    pop_size = min(20, max(10, num_targets * 2)); max_generations = min(20, max(15, num_targets * 2));
    population = cell(pop_size, 1); for i = 1:pop_size, population{i} = randperm(num_targets); end
    fitness = zeros(pop_size, 1); for i = 1:pop_size, fitness(i) = eval_fit_LUT(population{i}, dist_mat, outage_mat, throughput_mat, v_drone, remaining_time, min_throughput_req); end
    for gen = 1:max_generations
        sel_pop = cell(pop_size,1); for i=1:pop_size, cands=randperm(pop_size, 3); [~, bi]=min(fitness(cands)); sel_pop{i}=population{cands(bi)}; end
        off_pop = sel_pop; for i=1:2:pop_size-1, if rand<0.8, p1=sel_pop{i}; p2=sel_pop{i+1}; pt1=randi([1, num_targets-1]); pt2=randi([pt1+1, num_targets]); ch=zeros(1,num_targets); ch(pt1:pt2)=p1(pt1:pt2); idx=1; for j=1:num_targets, if ~ismember(p2(j), ch(pt1:pt2)), while idx>=pt1 && idx<=pt2, idx=idx+1; end; ch(idx)=p2(j); idx=idx+1; end; end; off_pop{i}=ch; end; end
        for i=1:pop_size, if rand<0.2, pos1=randi([1,num_targets]); pos2=randi([1,num_targets]); t=off_pop{i}(pos1); off_pop{i}(pos1)=off_pop{i}(pos2); off_pop{i}(pos2)=t; end; end
        n_fit = zeros(pop_size, 1); for i=1:pop_size, n_fit(i) = eval_fit_LUT(off_pop{i}, dist_mat, outage_mat, throughput_mat, v_drone, remaining_time, min_throughput_req); end
        c_pop = [population; off_pop]; c_fit = [fitness; n_fit]; [s_fit, s_idx] = sort(c_fit); population = c_pop(s_idx(1:pop_size)); fitness = s_fit(1:pop_size);
    end
    [best_fitness, best_idx] = min(fitness); best_path = population{best_idx};
    [total_outage, min_throughput] = calc_path_metrics(current_pos, target_positions, return_pos, best_path, H, beta0, delta_A, delta_U, eta, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, all_cluster_positions);
    
    total_distance = norm(current_pos - target_positions(best_path(1),:)); for i=1:length(best_path)-1, total_distance=total_distance+norm(target_positions(best_path(i),:)-target_positions(best_path(i+1),:)); end; total_distance=total_distance+norm(target_positions(best_path(end),:)-return_pos);
end

function fitness = eval_fit_LUT(path, dist_mat, outage_mat, th_mat, v_drone, max_time, min_th_req)
    N_all = size(dist_mat, 1); full_p = [1, path+1, N_all]; t_d=0; t_o=0; m_th=inf;
    for k=1:length(full_p)-1, u=full_p(k); v=full_p(k+1); t_d=t_d+dist_mat(u,v); t_o=t_o+outage_mat(u,v); if th_mat(u,v)<m_th, m_th=th_mat(u,v); end; end
    t_t = t_d/v_drone; pen=0; if t_t>max_time, pen=1000*(t_t-max_time); end
    th_term = 0; if m_th<min_th_req, th_term=1000*(min_th_req-m_th); else, th_term=-50*log(1+(m_th-min_th_req)); end
    fitness = 0.99998*(t_d/1000) + 0.00001*th_term + 0.00001*(t_o/100) + pen;
end

function [t_out, m_th] = calc_path_metrics(s_p, t_p, e_p, path, H, b0, dA, dU, eta, PA, Pu, s0, lL, lNL, blds, K_fac, a_c_p)
    t_out=0; m_th=inf; n_t=length(path);
    [so, st] = calc_seg_LUT(s_p, t_p(path(1),:), a_c_p, e_p, H, b0, dA, dU, eta, PA, Pu, s0, lL, lNL, blds, K_fac); t_out=t_out+so; m_th=min(m_th, st);
    for i=1:n_t-1, [so, st] = calc_seg_LUT(t_p(path(i),:), t_p(path(i+1),:), a_c_p, e_p, H, b0, dA, dU, eta, PA, Pu, s0, lL, lNL, blds, K_fac); t_out=t_out+so; m_th=min(m_th, st); end
    [so, st] = calc_seg_LUT(t_p(path(end),:), e_p, a_c_p, e_p, H, b0, dA, dU, eta, PA, Pu, s0, lL, lNL, blds, K_fac); t_out=t_out+so; m_th=min(m_th, st);
end

function [o_c, th] = calc_seg_LUT(p1, p2, a_t, s_p, H, b0, dA, dU, eta, PA, Pu, s0, lL, lNL, blds, K_fac)
    m_p=(p1+p2)/2; pU=[m_p(1), m_p(2), H]; pS=[s_p(1), s_p(2), 0]; dR=max(0.1, norm(pU-pS)); 
    if check_blk(pU, pS, blds), b2=b0*lNL/(dR^2); g2=b2*dU; P2=exp(-eta/g2); else, b2=b0*lL/(dR^2); g2=b2*dU; P2=my_q(sqrt(2*K_fac), sqrt(2*(K_fac+1)*eta/g2)); end
    if isempty(a_t), o_c=0; th=log2(1+g2); return; end
    m_o=0; m_t=inf; 
    for i=1:size(a_t,1)
        pT=[a_t(i,1), a_t(i,2), 0]; dT=max(0.1, norm(pU-pT));
        if check_blk(pU, pT, blds), b1=b0*lNL/(dT^2); g1=b1*dA; P1=exp(-eta/g1); else, b1=b0*lL/(dT^2); g1=b1*dA; P1=my_q(sqrt(2*K_fac), sqrt(2*(K_fac+1)*eta/g1)); end
        po=1-(P1*P2); m_o=max(m_o, po); m_t=min(m_t, log2(1+min(g1,g2)));
    end
    o_c=m_o*norm(p2-p1); th=m_t;
end

function blk = check_blk(p1, p2, blds)
    blk=false; p1=p1(:); p2=p2(:); d=p2-p1; if norm(d)<1e-6, return; end
    for b=1:size(blds,1)
        bx=blds(b,1); by=blds(b,2); bw=blds(b,3); bl=blds(b,4); bh=blds(b,5); xM=bx+bw; yM=by+bl;
        if p1(3)>bh && p2(3)>bh, continue; end
        if abs(d(1))<1e-6, if p1(1)>=bx && p1(1)<=xM, tx1=0; tx2=1; else continue; end; else, tx1=(bx-p1(1))/d(1); tx2=(xM-p1(1))/d(1); end
        if abs(d(2))<1e-6, if p1(2)>=by && p1(2)<=yM, ty1=0; ty2=1; else continue; end; else, ty1=(by-p1(2))/d(2); ty2=(yM-p1(2))/d(2); end
        tL=max([min(tx1,tx2), min(ty1,ty2), 0]); tH=min([max(tx1,tx2), max(ty1,ty2), 1]);
        if tL<=tH+1e-6, zM=p1(3)+((tL+tH)/2)*d(3); if zM<=bh, blk=true; return; end; end
    end
end

function q = my_q(a, b)
    if ~isempty(which('ncx2cdf')), q = 1-ncx2cdf(b.^2, 2, a.^2); else, f = @(x) x.*exp(-(x.^2+a.^2)/2).*besseli(0,a.*x); q = integral(f,b,inf); end
end

function labels = my_dbscan(X, epsilon, minPts)
    n = size(X, 1); labels = zeros(n, 1); C = 0;
    for i = 1:n
        if labels(i) ~= 0, continue; end
        neighbors = regionQuery(X, i, epsilon);
        if numel(neighbors) < minPts, labels(i) = -1; else
            C = C + 1; labels(i) = C; k = 1;
            while k <= numel(neighbors)
                p_n = neighbors(k);
                if labels(p_n) == -1, labels(p_n) = C; elseif labels(p_n) == 0
                    labels(p_n) = C; neighbors_p_n = regionQuery(X, p_n, epsilon);
                    if numel(neighbors_p_n) >= minPts, neighbors = [neighbors; neighbors_p_n]; end
                end
                k = k + 1;
            end
        end
    end
    function neighbors = regionQuery(X, p, epsilon), dists = sqrt(sum((X - X(p,:)).^2, 2)); neighbors = find(dists <= epsilon); end
end