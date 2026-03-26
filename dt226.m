
clear; clc; close all;

H = 200; num_Tx = 48; area_size = 1200; eta = 0.5; beta0 = 1e8; 
PA = 1000; Pu = 1000; sigma0 = 2.2; 
delta_A = PA / (sigma0^2); delta_U = Pu / (sigma0^2);
v_drone = 16.67; max_time = 800; min_throughput_req = 5.3;
eta_LoS = 1.0; eta_NLoS = 35.0; 
loss_LoS = 10^(-eta_LoS/10); loss_NLoS = 10^(-eta_NLoS/10); 
K_fac = 10^(10/10); 

% ========== 实验扫参设定 ==========
fixed_rv = 30;                 % 固定视觉定位圆 30m
re_test_values = 10:10:50;     % 扫描初始误差圆半径
num_tests = length(re_test_values);
num_algos = 3;        
num_mc_runs = 10;              % 蒙特卡洛仿真次数 10 次


fprintf('========== 启动基准算法对比仿真 (评价指标: 平均中断概率) ==========\n');

%% ================== 蒙特卡洛主循环 ==================
for mc = 1:num_mc_runs
    fprintf('\n======================================================\n');
    fprintf('>>> 开始执行蒙特卡洛仿真 第 %d / %d 次环境 <<<\n', mc, num_mc_runs);
    fprintf('======================================================\n');
    
    % --- 1. 生成每次独立的 3D 城市与节点 ---
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

    % 聚类处理
    labels = dbscan(Tx_nominal_pos, 140, 3); num_clusters = max(labels);
    if num_clusters == 0
        fprintf('警告：本次随机环境未产生有效聚类，跳过...\n');
        avg_outage_results_all(mc, :, :) = NaN; 
        continue;
    end
    
    transfer_stations = zeros(num_clusters, 2); cluster_node_indices = cell(num_clusters, 1);
    for i = 1:num_clusters, cluster_points = Tx_nominal_pos(labels==i, :); transfer_stations(i, :) = mean(cluster_points, 1); cluster_node_indices{i} = find(labels==i); end
    noise_indices = find(labels==-1);
    for i = 1:length(noise_indices), noise_point = Tx_nominal_pos(noise_indices(i), :); dists = sqrt(sum((transfer_stations-noise_point).^2, 2)); [~, closest] = min(dists); labels(noise_indices(i)) = closest; cluster_node_indices{closest} = [cluster_node_indices{closest}; noise_indices(i)]; end
    for i = 1:num_clusters, transfer_stations(i, :) = mean(Tx_nominal_pos(cluster_node_indices{i}, :), 1); end

    % --- 2. 在当前城市中运行 3 种对比算法 ---
    algo_names = {'Proposed Scheme (DT+Vision+Comms GA)', 'Baseline 1: No DT Updates', 'Baseline 2: Pure Geo-GA (Dist Only)'};
    for a_idx = 1:num_algos
        fprintf('  运行算法 [%d/%d]: %s ...\n', a_idx, num_algos, algo_names{a_idx});
        for r_idx = 1:num_tests
            curr_re = re_test_values(r_idx);
            % 获取该算法在当前条件下的 平均中断概率
            avg_prob = run_mission_algo(num_clusters, cluster_node_indices, transfer_stations, Tx_nominal_pos, curr_re, fixed_rv, H, beta0, delta_A, delta_U, eta, v_drone, max_time, PA, Pu, sigma0, loss_LoS, loss_NLoS, buildings, K_fac, min_throughput_req, a_idx);
            avg_outage_results_all(mc, a_idx, r_idx) = avg_prob;
        end
    end
end

%% ========== 数据后处理与蒙特卡洛平均 ==========
fprintf('\n仿真完成，正在进行数据平均与绘图...\n');
valid_runs = ~isnan(avg_outage_results_all(:, 1, 1));
outage_avg_final = squeeze(mean(avg_outage_results_all(valid_runs, :, :), 1));

%% ========== 3. 绘制带有学术质感的对比图 ==========
figure('Color', 'w', 'Position', [100, 100, 850, 600]); hold on;
colors = {[0.85 0.15 0.20], [0.5 0.5 0.5], [0.0 0.45 0.74]}; % 红, 灰, 蓝
markers = {'o', 's', '^'}; 
lines = {'-', '--', '-.'};

for a_idx = 1:num_algos
    plot(re_test_values, outage_avg_final(a_idx, :), [lines{a_idx} markers{a_idx}], ...
        'Color', colors{a_idx}, 'LineWidth', 3.0, 'MarkerSize', 9, ...
        'MarkerFaceColor', 'w', 'DisplayName', algo_names{a_idx});
end

grid on; grid minor; 
ax = gca; ax.GridColor = [0.6 0.6 0.6]; ax.GridAlpha = 0.3;
set(gca, 'FontSize', 13, 'LineWidth', 1.2, 'FontName', 'Arial');

xlabel('Initial Error Radius r_e (m)', 'FontSize', 15, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Average Outage Probability', 'FontSize', 15, 'FontWeight', 'bold', 'FontName', 'Arial');
title(sprintf('Algorithm Performance Comparison (%d MC Runs)', num_mc_runs), 'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'Arial');

lgd = legend('Location', 'northwest', 'FontSize', 12, 'FontName', 'Arial');
lgd.EdgeColor = [0.8 0.8 0.8];
lgd.Color = 'w';

hold off;
fprintf('========== 图表已成功生成！==========\n');


%% =========================================================================
%% 底层运行引擎 (评价指标：时间加权平均中断概率)
%% =========================================================================
function avg_outage_prob = run_mission_algo(num_cls, cls_idx, stations, Tx_pos, re, rv, H, b0, dA, dU, eta, v, max_t, PA, Pu, sig0, l_LoS, l_NLoS, blds, K, min_th, algo_type)
    num_Tx = size(Tx_pos, 1);
    DT = struct('true_pos', num2cell(Tx_pos,2), 'nominal_pos', num2cell(Tx_pos,2), 'error_circle', num2cell(ones(num_Tx,1)*re), 'visited', num2cell(false(num_Tx,1)), 'detect_count', num2cell(zeros(num_Tx,1)));
    
    total_outage_time_integral = 0; % 记录：中断概率 在时间上的积分
    total_flight_time_all = 0;      % 记录：总飞行时间
    
    for c_idx = 1:num_cls
        c_nodes = cls_idx{c_idx}; N_c = length(c_nodes); s_pos = stations(c_idx, :);
        all_c_pos = zeros(N_c, 2); for i=1:N_c, all_c_pos(i,:) = DT(c_nodes(i)).nominal_pos; end
        u_pos = s_pos; vis = false(N_c, 1); t_flown = 0; c_out_time_int = 0; t_since_opt = 0; curr_tgt = [];
        
        while sum(vis) < N_c && t_flown < max_t
            if isempty(curr_tgt) || t_since_opt >= 2
                unvis = find(~vis); N_unvis = length(unvis); det_pos = zeros(N_unvis, 2);
                for i = 1:N_unvis
                    glob = c_nodes(unvis(i)); DT(glob).detect_count = DT(glob).detect_count + 1;
                    lambda = 0.1; if algo_type == 2, lambda = 0; end % Baseline 1: No DT (lambda=0)
                    curr_re = max(2, re * exp(-lambda * DT(glob).detect_count)); DT(glob).error_circle = curr_re;
                    ang = rand()*2*pi; r = sqrt(rand())*curr_re; det_pos(i, :) = DT(glob).nominal_pos + [r*cos(ang), r*sin(ang)];
                end
                
                best_p = optimize_algo_LUT(u_pos, s_pos, det_pos, H, b0, dA, dU, eta, v, max_t-t_flown, min_th, PA, Pu, sig0, l_LoS, l_NLoS, blds, K, all_c_pos, algo_type);
                if isempty(best_p), [~, n_idx] = min(sqrt(sum((det_pos-u_pos).^2, 2))); best_p = n_idx; end
                curr_tgt = unvis(best_p(1)); t_since_opt = 0;  
            end
            
            glob = c_nodes(curr_tgt);
            ang = rand()*2*pi; r = sqrt(rand())*DT(glob).error_circle; det_curr = DT(glob).nominal_pos + [r*cos(ang), r*sin(ang)];
            dist_to = norm(det_curr - u_pos);
            
            % 防死锁机制：如果到达假目标依然未触发视觉，则原地盘旋重算
            if dist_to < 0.1
                curr_tgt = []; t_since_opt = 2; 
                % 盘旋0.5秒也要计算通信代价！
                [~, ~, current_p_out] = calc_seg_vec(u_pos, u_pos, all_c_pos, s_pos, H, b0, dA, dU, eta, PA, Pu, sig0, l_LoS, l_NLoS, blds, K);
                c_out_time_int = c_out_time_int + current_p_out * 0.5; 
                t_flown = t_flown + 0.5;
                continue; 
            end
            
            t_step = min(2, dist_to/v); dir = (det_curr - u_pos)/norm(det_curr - u_pos);
            fly_d = v * t_step; n_pos = u_pos + dir * fly_d;
            
            hit_t = inf; hit_g = -1; hit_l = -1;
            unvis = find(~vis); a_q = fly_d^2;
            for i = 1:length(unvis)
                g = c_nodes(unvis(i)); f_v = u_pos - DT(g).nominal_pos;
                b_q = 2 * dot(f_v, n_pos-u_pos); c_q = dot(f_v, f_v) - rv^2;
                if a_q > 1e-6
                    disc = b_q^2 - 4*a_q*c_q;
                    if disc >= 0, t1 = (-b_q-sqrt(disc))/(2*a_q); t2 = (-b_q+sqrt(disc))/(2*a_q); valid_t = [];
                        if t1 >= -1e-6 && t1 <= 1+1e-6, valid_t=[valid_t, max(0,t1)]; end; if t2 >= -1e-6 && t2 <= 1+1e-6, valid_t=[valid_t, max(0,t2)]; end; if c_q <= 0, valid_t=[valid_t, 0]; end
                        if ~isempty(valid_t), min_t = min(valid_t); if min_t < hit_t, hit_t = min_t; hit_g = g; hit_l = unvis(i); end; end
                    end
                elseif c_q <= 0, hit_t = 0; hit_g = g; hit_l = unvis(i); end
            end
            
            if hit_t <= 1.0
                h_pos = u_pos + hit_t * (n_pos-u_pos); act_t = t_step * hit_t;
                if fly_d * hit_t > 1e-3
                    [~, ~, current_p_out] = calc_seg_vec(u_pos, h_pos, all_c_pos, s_pos, H, b0, dA, dU, eta, PA, Pu, sig0, l_LoS, l_NLoS, blds, K);
                    c_out_time_int = c_out_time_int + current_p_out * act_t; 
                    t_flown = t_flown + act_t;
                end
                u_pos = DT(hit_g).true_pos; vis(hit_l) = true; DT(hit_g).visited = true; curr_tgt = []; t_since_opt = 2; 
            else
                [~, ~, current_p_out] = calc_seg_vec(u_pos, n_pos, all_c_pos, s_pos, H, b0, dA, dU, eta, PA, Pu, sig0, l_LoS, l_NLoS, blds, K);
                c_out_time_int = c_out_time_int + current_p_out * t_step; 
                t_flown = t_flown + t_step; t_since_opt = t_since_opt + t_step; u_pos = n_pos;
            end
        end
        while norm(u_pos - s_pos) >= 1.0
            t_step = min(2, norm(u_pos - s_pos)/v); dir = (s_pos - u_pos)/norm(s_pos - u_pos); n_pos = u_pos + dir * (v*t_step);
            [~, ~, current_p_out] = calc_seg_vec(u_pos, n_pos, all_c_pos, s_pos, H, b0, dA, dU, eta, PA, Pu, sig0, l_LoS, l_NLoS, blds, K);
            c_out_time_int = c_out_time_int + current_p_out * t_step; 
            t_flown = t_flown + t_step; u_pos = n_pos;
        end
        
        total_outage_time_integral = total_outage_time_integral + c_out_time_int;
        total_flight_time_all = total_flight_time_all + t_flown;
    end
    
    % 计算平均中断概率 P_out_avg = Integral(P_out * dt) / T_total
    if total_flight_time_all > 0
        avg_outage_prob = total_outage_time_integral / total_flight_time_all;
    else
        avg_outage_prob = 0;
    end
end

function best_p = optimize_algo_LUT(c_pos, r_pos, t_pos, H, b0, dA, dU, eta, v, rem_t, m_th, PA, Pu, sig0, l_LoS, l_NLoS, blds, K, a_c_pos, a_type)
    N_t = size(t_pos, 1); if N_t == 0, best_p = []; return; end; if N_t == 1, best_p = 1; return; end
    N_all = N_t + 2; pts = [c_pos; t_pos; r_pos]; d_mat = zeros(N_all); o_mat = zeros(N_all); th_mat = inf(N_all);
    for i=1:N_all
        for j=1:N_all
            if i~=j
                d_mat(i,j)=norm(pts(i,:)-pts(j,:)); 
                [o_mat(i,j), th_mat(i,j), ~] = calc_seg_vec(pts(i,:), pts(j,:), a_c_pos, r_pos, H, b0, dA, dU, eta, PA, Pu, sig0, l_LoS, l_NLoS, blds, K); 
            end
        end
    end
    
    % GA Options
    pop_s = min(40, max(20, N_t*4)); gen = min(50, max(30, N_t*4));
    pop = cell(pop_s, 1); for i=1:pop_s, pop{i}=randperm(N_t); end; fit = zeros(pop_s, 1);
    for i=1:pop_s, fit(i) = eval_fit(pop{i}, d_mat, o_mat, th_mat, v, rem_t, m_th, a_type); end
    for g=1:gen
        sel = cell(pop_s,1); for i=1:pop_s, cands=randperm(pop_s, 3); [~, bi]=min(fit(cands)); sel{i}=pop{cands(bi)}; end
        off = sel; for i=1:2:pop_s-1, if rand<0.8, p1=sel{i}; p2=sel{i+1}; pt1=randi([1, N_t-1]); pt2=randi([pt1+1, N_t]); ch=zeros(1,N_t); ch(pt1:pt2)=p1(pt1:pt2); idx=1; for j=1:N_t, if ~ismember(p2(j), ch(pt1:pt2)), while idx>=pt1 && idx<=pt2, idx=idx+1; end; ch(idx)=p2(j); idx=idx+1; end; end; off{i}=ch; end; end
        for i=1:pop_s, if rand<0.2, pos1=randi([1, N_t]); pos2=randi([1, N_t]); tmp=off{i}(pos1); off{i}(pos1)=off{i}(pos2); off{i}(pos2)=tmp; end; end
        n_fit = zeros(pop_s, 1); for i=1:pop_s, n_fit(i) = eval_fit(off{i}, d_mat, o_mat, th_mat, v, rem_t, m_th, a_type); end
        comb_pop = [pop; off]; comb_fit = [fit; n_fit]; [~, s_idx] = sort(comb_fit); pop = comb_pop(s_idx(1:pop_s)); fit = comb_fit(s_idx(1:pop_s));
    end
    best_p = pop{1};
end

function fit = eval_fit(p, d_m, o_m, th_m, v, mt, min_th_req, a_type)
    N = size(d_m,1); fp = [1, p+1, N]; u = fp(1:end-1); vv = fp(2:end); idx = u + (vv-1)*N;
    t_d = sum(d_m(idx)); t_o = sum(o_m(idx)); m_th = min(th_m(idx));
    t_t = t_d/v; pen = 0; if t_t > mt, pen = 1000*(t_t - mt); end
    th_term = 0; if m_th < min_th_req, th_term = 1000*(min_th_req - m_th); else, th_term = -50*log(1+(m_th-min_th_req)); end
    
    % Baseline 2: Pure Geo (Dist only)
    if a_type == 3
        fit = t_d + pen; % 仅优化距离
    else
        fit = 0.6*(t_d/1000) + 0.2*th_term + 0.2*(t_o/100) + pen;
    end
end

% 返回第三个参数 mo，即这段链路的瞬时中断概率
function [o_c, th, mo] = calc_seg_vec(p1, p2, all_t, s_pos, H, b0, dA, dU, eta, PA, Pu, sig0, lL, lNL, blds, K)
    m_p = (p1+p2)/2; pU = [m_p(1), m_p(2), H]; pS = [s_pos(1), s_pos(2), 0]; d_rx = max(0.1, norm(pU-pS));
    d = pS-pU; blk = false; 
    for b=1:size(blds,1), bx=blds(b,1); by=blds(b,2); bw=blds(b,3); bl=blds(b,4); bh=blds(b,5);
        tx1=(bx-pU(1))/d(1); tx2=(bx+bw-pU(1))/d(1); ty1=(by-pU(2))/d(2); ty2=(by+bl-pU(2))/d(2);
        tmin = max([0, min(tx1,tx2), min(ty1,ty2)]); tmax = min([1, max(tx1,tx2), max(ty1,ty2)]);
        if tmin<=tmax+1e-6 && pU(3)+((tmin+tmax)/2)*d(3)<=bh, blk=true; break; end
    end
    if blk, g2 = b0*lNL/(d_rx^2)*dU; P2 = exp(-eta/g2); else, g2 = b0*lL/(d_rx^2)*dU; P2 = my_q(sqrt(2*K), sqrt(2*(K+1)*eta/g2)); end
    
    if isempty(all_t)
        mo = 1 - P2; 
        o_c = mo * norm(p2-p1); 
        th = log2(1+g2); 
        return; 
    end
    
    mo=0; mt=inf;
    for i=1:size(all_t,1), pT=[all_t(i,1), all_t(i,2), 0]; dT=max(0.1, norm(pU-pT)); dd=pT-pU; bl=false;
        for b=1:size(blds,1), bx=blds(b,1); by=blds(b,2); bw=blds(b,3); bl_b=blds(b,4); bh=blds(b,5);
            tx1=(bx-pU(1))/dd(1); tx2=(bx+bw-pU(1))/dd(1); ty1=(by-pU(2))/dd(2); ty2=(by+bl_b-pU(2))/dd(2);
            tmin = max([0, min(tx1,tx2), min(ty1,ty2)]); tmax = min([1, max(tx1,tx2), max(ty1,ty2)]);
            if tmin<=tmax+1e-6 && pU(3)+((tmin+tmax)/2)*dd(3)<=bh, bl=true; break; end
        end
        if bl, g1 = b0*lNL/(dT^2)*dA; P1 = exp(-eta/g1); else, g1 = b0*lL/(dT^2)*dA; P1 = my_q(sqrt(2*K), sqrt(2*(K+1)*eta/g1)); end
        po = 1 - P1*P2; if po>mo, mo=po; end; ct = log2(1+min(g1,g2)); if ct<mt, mt=ct; end
    end
    o_c = mo*norm(p2-p1); th = mt;
end

function q = my_q(a, b) 
    if ~isempty(which('ncx2cdf')), q=1-ncx2cdf(b.^2, 2, a.^2); else, f=@(x) x.*exp(-(x.^2+a.^2)/2).*besseli(0,a.*x); q=integral(f,b,inf); end
end