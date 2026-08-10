function [y, t0_ref] = sim_rx_from_scatterers_perTX(P, A, TX, RX, f0, c, fs, excitation, opts)
% sim_rx_from_scatterers_perTX
% 纯整数时延的超声回波合成（Born 近似，单次散射），支持每个发射阵元不同激励。
%
% 输入
%   P   : [K x 3]  散射体坐标 (m)
%   A   : [K x 1]  散射体幅值/散射系数（线性幅度）
%   TX  : [Nt x 3] 发射阵元坐标 (m)（默认“同时触发”，但信号可不同）
%   RX  : [Nr x 3] 接收阵元坐标 (m)
%   f0  : 中心频率 (Hz)（此简化模型不直接使用；保留作记录）
%   c   : 声速 (m/s)
%   fs  : 采样率 (Hz)
%   excitation : [Nt x M] 各发射阵元的时域激励（每行一个 TX；行向量或矩阵）
%   opts.round : 'round'(默认) | 'floor' | 'ceil'  时延→样点取整
%   opts.precision : 'single'(默认) | 'double'
%   opts.delay : 标量或 [K x 1]，对每个散点附加的额外时延（秒）
%   opts.atten : 'none'(默认) | 'twoway_R' | 'sqrt_twoway_R'
%                - 'twoway_R'        幅度 ∝ A / (Rtx*Rrx)
%                - 'sqrt_twoway_R'   幅度 ∝ A / sqrt(Rtx*Rrx)
%
% 输出
%   y       : [M+Lir-1 x Nr] 各通道接收信号（各 TX 卷积后求和；最早到达对齐到 t=0）
%   t0_ref  : 最早到达的绝对时延(秒)
%
% 复杂度 O(K*Nt*Nr)。Nt 或 K 很大时注意计算量与内存。

    if nargin < 9 || ~isfield(opts,'round'),   opts.round = 'round'; end
    if ~isfield(opts,'precision'),             opts.precision = 'single'; end
    if ~isfield(opts,'atten'),                 opts.atten = 'none'; end
    if ~isfield(opts,'delay'),                 opts.delay = 0; end

    % 统一类型
    P  = double(P);   A  = double(A(:));
    TX = double(TX);  RX = double(RX);
    fs = double(fs)*10;  c  = double(c);
    excitation = resample(excitation',10,1)';
    % 维度
    [Nt, ~] = size(TX);
    [Nr, ~] = size(RX);
    K       = size(P,1);

    % 处理 excitation 尺寸/方向：期望 [Nt x M]
    if size(excitation,1) ~= Nt && size(excitation,2) == Nt
        excitation = excitation.';  % 如果给的是 [M x Nt]，则转置为 [Nt x M]
    end
    assert(size(excitation,1) == Nt, 'excitation 大小应为 [Nt x M] 或其转置');
    Mx = size(excitation,2);

    % delay 归一化为 [K x 1]
    if isscalar(opts.delay)
        delayK = repmat(double(opts.delay), K, 1);
    else
        delayK = double(opts.delay(:));
        assert(numel(delayK) == K, 'opts.delay 尺寸应为标量或 [K x 1]');
    end

    % -------- Pass 1：统计最早/最晚到达（确定 IR 长度与 t0_ref） --------
    t_min = inf;  t_max = -inf;
    for r = 1:Nr
        Rr = RX(r,:);
        for k = 1:K
            Pk  = P(k,:);
            Rrx = norm(Pk - Rr);               % 散点→该接收阵元
            Rtx = sqrt(sum((TX - Pk).^2, 2));  % 所有发射→散点 [Nt x 1]
            tau_vec = (Rtx + Rrx) / c + delayK(k);  % [Nt x 1]
            % 更新全局最早/最晚
            t_min = min(t_min, min(tau_vec));
            t_max = max(t_max, max(tau_vec));
        end
    end
    t0_ref = t_min;

    % 取整函数
    switch lower(opts.round)
        case 'round', idxfun = @(t) round(t*fs);
        case 'floor', idxfun = @(t) floor(t*fs);
        case 'ceil',  idxfun = @(t) ceil(t*fs);
        otherwise, error('opts.round 仅支持 round/floor/ceil');
    end

    % IR长度（相对零点的最大样点 + 1）
    Lir = idxfun(t_max - t0_ref) + 1;

    % 输出长度
    Mout = Lir + Mx - 1;
    y = zeros(Mout, Nr, opts.precision);

    % 精度转换
    x_all = cast(excitation, opts.precision);  % [Nt x Mx]

    % -------- Pass 2：逐接收通道，按 TX 累加稀疏 IR，再卷积并求和 --------
    for r = 1:Nr
        Rr = RX(r,:);

        % 为该接收通道准备按 TX 分解的 IR： [Lir x Nt]
        ir_tx = zeros(Lir, Nt, opts.precision);

        for k = 1:K
            Pk  = P(k,:);
            Rrx = norm(Pk - Rr);               % 散点→该接收阵元
            Rtx = sqrt(sum((TX - Pk).^2, 2));  % [Nt x 1] 所有 TX→散点

            % 到达时延（绝对）+ 取整为相对零点的样点索引
            tau = (Rtx + Rrx) / c + delayK(k);     % [Nt x 1]
            n   = idxfun(tau - t0_ref);            % [Nt x 1]
            n(n < 0) = 0;                          % 数值保险

            % 衰减
            switch lower(opts.atten)
                case 'none'
                    w = A(k) * ones(Nt,1);  % 每个 TX 同幅（仅由 A 决定）
                case 'twoway_r'
                    w = (A(k) ./ (Rtx .* Rrx));     % 双程 1/R 衰减
                case 'sqrt_twoway_r'
                    w = (A(k) ./ sqrt(Rtx .* Rrx)); % 幅度随 sqrt 衰减
                otherwise
                    error('opts.atten 仅支持 none / twoway_r / sqrt_twoway_r');
            end

            % 累加到按 TX 分解的冲激响应
            % （注意 n 是从 0 开始的相对索引，这里 +1 转为 MATLAB 下标）
            for j = 1:Nt
                ir_tx(n(j)+1, j) = ir_tx(n(j)+1, j) + cast(w(j), opts.precision);
            end
        end

        % 与每个 TX 的激励分别卷积并求和：y(:,r) = sum_j conv(x_j, ir_tx(:,j))
        yr = zeros(Mout, 1, opts.precision);
        for j = 1:Nt
            if any(ir_tx(:,j))
                % 确保使用列向量卷积
                xj = x_all(j, :).';
                yr = yr + conv(xj, ir_tx(:,j), 'full');
            end
        end

        y(:, r) = yr;
    end
    y = resample(y,1,10);
end
