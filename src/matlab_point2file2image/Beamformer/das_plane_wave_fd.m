function [beam, ranges_out] = das_plane_wave_fd(cfg, rx)
    % Plane-wave DAS (per-angle FD-TTD)
    % 先全角度计算最大/最小时延，确定统一零填充；再逐角度波束形成。
    %
    % Inputs:
    %   cfg.t0, cfg.fs, cfg.c, cfg.rx_xyz, cfg.angles [, cfg.ranges]
    %   cfg.fd.nfft   : FFT 长度（若未给，则用 2^nextpow2(信号长度+填充)）
    %   cfg.fd.sign   : +1/-1（默认 +1；主瓣方向相反可设 -1）
    %   cfg.w         : 阵元固定权重 [M×1]（默认全 1）
    %   cfg.f0        : 中心频率 (Hz)，用于载频相位补偿
    %   cfg.cf.enable : true/false（默认 false）
    %   cfg.cf.mode   : 'cf' | 'pcf' | 'gcf' | 'mcf' | 'slsc'   % 默认 'cf'
    %   cfg.cf.gamma  : >0 权重指数（默认 1.0）
    %   cfg.cf.eps    : 稳定项（默认 1e-12）
    %   cfg.cf.p      : GCF 幂指数 p（默认 2；p=2 退化为 CF）
    %   cfg.cf.L      : SLSC 短延迟最大滞后（默认 min(16, floor(M/4))）
    %
    % Outputs:
    %   img        : [Nr×Na]（若提供 cfg.ranges，否则 []）
    %   beam       : [NFFT×Na] 每角度一整条波束时序
    %   ranges_out : [NFFT×1] 与 beam 第一维匹配的距离轴：c*(t0 + (0:NFFT-1)/fs)/2
    
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
        w = ones(M,1);
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
    
    % -------- BW 检查（用于频域滤波）--------
    if ~isfield(cfg,'BW') || isempty(cfg.BW)
        cfg.BW = fs;  % 默认使用全带宽
    end
    
    % ================================================================
    % ① 全角度预计算“样点域时延”范围（决定统一零填充）
    %    d_samp(ia,m) = sgn * fs * (rx_xyz(m,:)·u(ia))/c
    % ================================================================
    ux = sin(angles);             % 1×Na
    uz = cos(angles);             % 1×Na
    dx = rx_xyz(:,1);             % M×1
    dz = rx_xyz(:,3);             % M×1
    % M×Na：对所有角度/阵元的样点时延
    d_samp_all = sgn * (fs/c) * (dx*ux + dz*uz);
    min_d = min(d_samp_all(:));
    max_d = max(d_samp_all(:));
    
    min_int = floor(min_d);
    max_int = ceil(max_d);
    
    pad_left  = max(0, -min_int);         % 使最小整数时移后起始不越界
    pad_right = max(0,  max_int);         % 使最大整数时移后结尾不越界
    Lsig = round(Nsamp) + pad_left + pad_right;  % 统一的时域长度（移位后不截断）
    
    % -------- NFFT & 输出距离轴（与 beam 匹配）--------
    if ~isfield(cfg,'fd') || ~isfield(cfg.fd,'nfft') || isempty(cfg.fd.nfft)
        NFFT = 2^nextpow2(Lsig);
    else
        NFFT = max(Lsig, max(16, round(cfg.fd.nfft))); % 至少覆盖信号长度
    end
    ranges_out = c * ( t0 + ((0:NFFT-1).'/fs) ) / 2-(pad_left)/2/fs*c;  % 若是一程距离，去掉“/2”
    
    % -------- 预分配 / 频域指数 --------
    beam = complex(zeros(NFFT, Na));
    kvec = zeros(NFFT, 1);
    f_vector = [-NFFT/2:NFFT/2-1]/NFFT*fs;
    index = find((f_vector > -cfg.BW/2) & (f_vector < cfg.BW/2));
    kvec(index) = f_vector(index);
    kvec = fftshift(kvec);
    
    % k = (0:NFFT-1).';                                % 0..NFFT-1
    % ================================================================
    % ② 逐角度波束形成（不使用 ranges 于中间流程）
    % ================================================================
    for ia = 1:Na
        % if mod(ia-1, 100) == 0
        %     fprintf('Processing angle %d / %d\n', ia-1, Na);
        % end
        % 当前角度的样点域时延（连续）
        d_samp = sgn * (fs/c) * (rx_xyz * [ux(ia); 0; uz(ia)]);  % [M×1]
        n_int  = round(d_samp);                                  % 整数样点
        d_res  = d_samp - n_int;                                 % 分数残差
    
        % —— 统一零填充后的整数时移（无截断、无循环）——
        x_shift = zeros(Lsig, M, 'like', rx);
        % 将每路 rx 放入 [1+pad_left+n_int(m) : pad_left+n_int(m)+Nsamp]
        starts = 1 + pad_left + n_int(:)+round(0*Nsamp);
        ends   = starts + Nsamp - 1;
        % （预计算 pad 足够，索引必落在 1..Lsig 之内）
        for m = 1:M
            x_shift(starts(m):ends(m), m) = rx(:,m);
        end
    
        % —— 全通道一次 FFT —— 
        X = fft(x_shift, NFFT, 1);                               % [NFFT×M]
    
        % —— 残差相移（频域线性相位）——
        phase_fix = exp(-1j * 2*pi * cfg.f0 * (d_samp(:).')/fs);
        phase_tbl = exp(-1j * 2*pi * kvec * (d_res(:).')/fs); % [NFFT×M]
        X_aligned = X .* phase_tbl.*phase_fix;
    
        % —— CF 计算（若启用）——
        if cf_enable
            % 转换到时域以计算 CF
            x_aligned = ifft(X_aligned, NFFT, 1);  % [NFFT×M]
            
            % 计算 CF 统计量
            absX = abs(x_aligned);                  % [NFFT×M]
            s_coh = sum(x_aligned, 2);             % [NFFT×1] Σ x_m
            s_abs2 = sum(absX.^2, 2);              % [NFFT×1] Σ |x_m|^2
            s_abs1 = sum(absX, 2);                  % [NFFT×1] Σ |x_m| (MCF)
            Meff = M;                               % 有效通道数（全通道对齐后）
            
            % 根据模式计算 CF
            switch lower(cf_mode)
                case 'cf'   % |Σx|^2 / (Meff * Σ|x|^2)
                    cf_fac = (abs(s_coh).^2) ./ (Meff .* s_abs2 + cf_eps);
                    
                case 'pcf'  % | (1/Meff) * Σ e^{j∠x} |
                    denom = max(absX, cf_eps);
                    ph = x_aligned ./ denom;
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
                        Csum = zeros(NFFT, 1, 'like', x_aligned);
                        for k = 1:slsc_L
                            if k < M
                                X1 = x_aligned(:, k+1:M);
                                X2 = x_aligned(:, 1:M-k);
                                num = sum(abs(X1 .* conj(X2)), 2);
                                den = sum(abs(X1).*abs(X2), 2) + cf_eps;
                                Ck = num ./ den;
                                Csum = Csum + Ck;
                            end
                        end
                        cf_fac = Csum / slsc_L;
                    else
                        cf_fac = ones(NFFT, 1, 'like', x_aligned);
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
            cf_fac = ones(NFFT, 1, 'like', X_aligned);
        end
    
        % —— 阵元加权求和 & IFFT —— 
        Y_f = X_aligned * w;                                     % [NFFT×1]
        beam(:, ia) = ifft(Y_f, NFFT, 1) .* cf_fac;             % [NFFT×1] 应用 CF
    end
    % fprintf('Beamforming completed\n');
    end
    % ================== 辅助：安全取字段默认值 ==================
    function v = getfield_default(s, name, defaultv)
        if isfield(s,name) && ~isempty(s.(name))
            v = s.(name);
        else
            v = defaultv;
        end
    end
    