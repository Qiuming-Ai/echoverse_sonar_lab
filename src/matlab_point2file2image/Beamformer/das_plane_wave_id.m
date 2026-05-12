function [beam, ranges_out] = das_plane_wave_id(cfg, rx)
    % Plane-wave DAS (Integer Delay only, no FFT/IFFT)
    % 使用配置的计算采样频率进行整数时延，然后重采样回原频率
    %
    % Inputs:
    %   cfg.t0, cfg.fs, cfg.c, cfg.rx_xyz, cfg.angles [, cfg.ranges]
    %   cfg.fs_calc   : 计算采样频率 (Hz)，用于整数时延（默认 1e6）
    %   cfg.fd.sign   : +1/-1（默认 +1；主瓣方向相反可设 -1）
    %   cfg.w         : 阵元固定权重 [M×1]（默认全 1）
    %   cfg.f0        : 中心频率 (Hz)，用于载频相位补偿（可选）
    %   cfg.cf.enable : true/false（默认 false）
    %   cfg.cf.mode   : 'cf' | 'pcf' | 'gcf' | 'mcf' | 'slsc'   % 默认 'cf'
    %   cfg.cf.gamma  : >0 权重指数（默认 1.0）
    %   cfg.cf.eps    : 稳定项（默认 1e-12）
    %   cfg.cf.p      : GCF 幂指数 p（默认 2；p=2 退化为 CF）
    %   cfg.cf.L      : SLSC 短延迟最大滞后（默认 min(16, floor(M/4))）
    %
    % Outputs:
    %   beam       : [Nsamp_out×Na] 每角度一整条波束时序
    %   ranges_out : [Nsamp_out×1] 与 beam 第一维匹配的距离轴
    
    % -------- 基本参数 --------
    if ~isfield(cfg,'c')  || isempty(cfg.c),  cfg.c  = 1500;  end
    if ~isfield(cfg,'t0') || isempty(cfg.t0), cfg.t0 = 0;     end
    t0 = cfg.t0; fs = cfg.fs; c = cfg.c;
    angles = -cfg.angles(:).';                        % 与先前实现保持一致：取反

    [Nsamp, M] = size(rx);
    rx_xyz = cfg.rx_xyz;
    if size(rx_xyz,2)==2, rx_xyz = [rx_xyz, zeros(M,1)]; end
    rx_xyz = double(rx_xyz);
    Na = numel(angles);
    
    % -------- sign / 权重 --------
    if ~isfield(cfg,'fd') || ~isfield(cfg.fd,'sign') || isempty(cfg.fd.sign)
        sgn = +1;
    else
        sgn = sign(cfg.fd.sign);
    end
    if isfield(cfg,'w') && ~isempty(cfg.w)
        w = cfg.w(:);
        if numel(w) ~= M, error('cfg.w 大小应为 M 元素'); end
    else
        % w = ones(M,1);
        w = hamming(M);
    end
    
    % -------- CF 家族选项 --------
    if ~isfield(cfg,'cf') || ~isfield(cfg.cf,'enable') || ~cfg.cf.enable
        cf_enable = false;
        cf_mode   = 'cf';
        cf_gamma  = 1.0;
        cf_eps    = 1e-12;
        gcf_p     = 2;
        slsc_L    = 0;
    else
        cf_enable = true;
        cf_mode   = getfield_default(cfg.cf,'mode','cf');   % 'cf' | 'pcf' | 'gcf' | 'mcf' | 'slsc'
        cf_gamma  = getfield_default(cfg.cf,'gamma',1.0);
        cf_eps    = getfield_default(cfg.cf,'eps',1e-12);
        gcf_p     = getfield_default(cfg.cf,'p',2);
        slsc_L    = getfield_default(cfg.cf,'L', min(16, max(1,floor(M/4))));
        slsc_L    = min(slsc_L, max(0,M-1));  % L ≤ M-1
    end
    
    % -------- f0 检查（用于载频相位补偿）--------
    if ~isfield(cfg,'f0') || isempty(cfg.f0)
        cfg.f0 = 0;  % 默认不使用载频相位补偿
    end
    
    % -------- fs_calc 检查（用于整数时延的计算采样频率）--------
    if ~isfield(cfg,'fs_calc') || isempty(cfg.fs_calc)
        fs_calc = cfg.fs;  % 默认计算采样频率 1MHz
    else
        fs_calc = cfg.fs_calc;
    end
    Nsamp_new = Nsamp;
    rx_new = rx;
    if fs_calc~=cfg.fs

    fprintf('Original fs: %.2f Hz, Calculation fs: %.2f Hz (ratio: %.2f)\n', fs, fs_calc, fs_calc/fs);
    
    % ================================================================
    % ① 使用 resample 将信号转变为计算采样频率
    % ================================================================
    % 计算重采样参数（使用有理数比）
    [P, Q] = rat(fs_calc/fs, 1e-6);  % 找到最接近的有理数比
    
    % 重采样到计算频率
    Nsamp_new = round(Nsamp * P / Q);
    rx_new = zeros(Nsamp_new, M, 'like', rx);
    for m = 1:M
        rx_new(:, m) = resample(rx(:, m), P, Q);
    end
    end
    % 更新采样频率
    fs_work = fs_calc;  % 工作采样频率
    
    % ================================================================
    % ② 在计算采样频率下计算所有角度的整数时延
    % ================================================================
    % 计算所有角度下的时延范围，确定需要的填充
    dx = rx_xyz(:,1);  % M×1
    dz = rx_xyz(:,3);  % M×1
    ux = sin(angles);             % 1×Na
    uz = cos(angles);             % 1×Na
    % M×Na：对所有角度/阵元的样点时延（在计算采样频率下）
    d_samp_all = sgn * (fs_work/c) * (dx*ux + dz*uz);
    min_d = min(d_samp_all(:));
    max_d = max(d_samp_all(:));
    
    min_int = floor(min_d);
    max_int = ceil(max_d);
    
    pad_left  = max(0, -min_int);         % 使最小整数时移后起始不越界
    pad_right = max(0,  max_int);         % 使最大整数时移后结尾不越界
    Lsig = Nsamp_new + pad_left + pad_right;  % 统一的时域长度（移位后不截断）
    
    % 预分配输出
    beam_work = complex(zeros(Lsig, Na));
    
    % 逐角度波束形成
    for ia = 1:Na
        if mod(ia-1, 100) == 0
            fprintf('Processing angle %d / %d\n', ia-1, Na);
        end
        
        % 当前角度的样点域时延（在新采样频率下）
        d_samp = sgn * (fs_work/c) * (rx_xyz * [ux(ia); 0; uz(ia)]);  % [M×1]
        n_int  = round(d_samp);                                  % 整数样点
        
        % —— 统一零填充后的整数时移（无截断、无循环）——
        x_shift = zeros(Lsig, M, 'like', rx_new);
        % 将每路 rx_new 放入 [1+pad_left+n_int(m) : pad_left+n_int(m)+Nsamp_new]
        starts = 1 + pad_left + n_int(:);
        ends   = starts + Nsamp_new - 1;
        % （预计算 pad 足够，索引必落在 1..Lsig 之内）
        for m = 1:M
            if starts(m) >= 1 && ends(m) <= Lsig
                x_shift(starts(m):ends(m), m) = rx_new(:,m);
            end
        end
        
        % —— 载频相位补偿（如果启用）——
        if cfg.f0 ~= 0
            % 计算相位补偿
            phase_comp = exp(-1j * 2*pi * cfg.f0 * (d_samp(:).')/fs_work);
            % 应用相位补偿（在时域通过复数乘法）
            for m = 1:M
                x_shift(:, m) = x_shift(:, m) * phase_comp(m);
            end
        end
        
        % —— CF 计算（若启用）——
        if cf_enable
            % 计算 CF 统计量
            absX = abs(x_shift);                  % [Lsig×M]
            s_coh = sum(x_shift, 2);             % [Lsig×1] Σ x_m
            s_abs2 = sum(absX.^2, 2);              % [Lsig×1] Σ |x_m|^2
            s_abs1 = sum(absX, 2);                  % [Lsig×1] Σ |x_m| (MCF)
            Meff = M;                               % 有效通道数（全通道对齐后）
            
            % 根据模式计算 CF
            switch lower(cf_mode)
                case 'cf'   % |Σx|^2 / (Meff * Σ|x|^2)
                    cf_fac = (abs(s_coh).^2) ./ (Meff .* s_abs2 + cf_eps);
                    
                case 'pcf'  % | (1/Meff) * Σ e^{j∠x} |
                    denom = max(absX, cf_eps);
                    ph = x_shift ./ denom;
                    ph_sum = sum(ph, 2);
                    cf_fac = abs(ph_sum) ./ Meff;
                    
                case 'gcf'  % (|Σx|^p) / (Meff^(p-1) * Σ|x|^p)
                    num = abs(s_coh).^gcf_p;
                    den = (Meff.^(max(gcf_p-1,0))) .* (sum(absX.^gcf_p, 2) + cf_eps);
                    cf_fac = num ./ den;
                    
                case 'mcf'  % |Σx|^2 / ( (Σ|x|)^2 )
                    den = (s_abs1.^2) + cf_eps;
                    cf_fac = (abs(s_coh).^2) ./ den;
                    
                case 'slsc' % mean_k ⟨ | x_m x_{m-k}* | / (|x_m||x_{m-k}|) ⟩
                    if slsc_L > 0
                        Csum = zeros(Lsig, 1, 'like', x_shift);
                        for k = 1:slsc_L
                            if k < M
                                X1 = x_shift(:, k+1:M);
                                X2 = x_shift(:, 1:M-k);
                                num = sum(abs(X1 .* conj(X2)), 2);
                                den = sum(abs(X1).*abs(X2), 2) + cf_eps;
                                Ck = num ./ den;
                                Csum = Csum + Ck;
                            end
                        end
                        cf_fac = Csum / slsc_L;
                    else
                        cf_fac = ones(Lsig, 1, 'like', x_shift);
                    end
                    
                otherwise
                    error('未知 cfg.cf.mode: %s', cf_mode);
            end
            
            % 应用 gamma 指数
            if cf_gamma ~= 1
                cf_fac = cf_fac .^ cf_gamma;
            end
            % 裁剪到 [0,1]
            cf_fac = min(max(real(cf_fac), 0), 1);
        else
            cf_fac = ones(Lsig, 1, 'like', x_shift);
        end
        
        % —— 阵元加权求和 —— 
        beam_work(:, ia) = (x_shift * w) .* cf_fac;  % [Lsig×1] 应用 CF
    end
    
    % ================================================================
    % ⑥ 使用 resample 转变为原采样频率
    % ================================================================
    % 计算重采样回原频率
    % 从 fs_work 回到 fs
    [P_back, Q_back] = rat(fs/fs_work, 1e-6);  % 找到最接近的有理数比
    
    % 输出长度固定为原信号长度+30
    Nsamp_out = Nsamp + 30;
    beam = complex(zeros(Nsamp_out, Na));
    
    % 对每个角度进行重采样
    for ia = 1:Na
        beam_resampled = resample(beam_work(:, ia), P_back, Q_back);
        
        % 确保输出长度精确等于 Nsamp_out（截断或填充）
        if length(beam_resampled) >= Nsamp_out
            beam(:, ia) = beam_resampled(1:Nsamp_out);
        else
            beam(1:length(beam_resampled), ia) = beam_resampled;
            beam(length(beam_resampled)+1:end, ia) = 0;
        end
    end
    
    % ================================================================
    % ⑦ 计算输出距离轴
    % ================================================================
    ranges_out = c * ( t0 + ((0:Nsamp_out-1).'/fs) ) / 2;
    
    fprintf('Beamforming completed\n');
end

% ================== 辅助：安全取字段默认值 ==================
function v = getfield_default(s, name, defaultv)
    if isfield(s,name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = defaultv;
    end
end

