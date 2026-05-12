function [excitation_nt,excitation_ori, delays_samp] = gen_multisubband_lfm_excitation( ...
        TX, angles_deg, fc, BW, pol, pw, fs, c, opts)
% gen_multisubband_lfm_excitation
% 多子带LFM脉冲 + 远场整数延时聚焦 → 每个阵元的发射时域序列
%
% 输入
%   TX         : [Nt x 3] 阵元坐标 (m)
%   angles_deg : [Na x 1] 子带瞄准的方位角（度）；0°=+z方向，+θ向+x偏
%   fc         : [Na x 1] 各子带中心频率 (Hz)
%   BW         : [Na x 1] 各子带带宽 (Hz)
%   pol        : [Na x 1] 'up' 或 'down'（上/下扫频）
%   pw         : [Na x 1] 各子带脉宽 (s)
%   fs         : 标量，采样率 (Hz)
%   c          : 标量，声速 (m/s)
%   opts       : 结构体，可选字段：
%                - round : 'round'(默认) | 'floor' | 'ceil'  延时取整方式
%                - win   : 'none'(默认) | 'hamming' 子带包络窗
%                - amp   : [Na x 1] 各子带幅度系数（默认全 1）
%                - pad   : 额外尾部零填充样点数（默认 0）
%
% 输出
%   excitation_nt : [Nt x Mout] 各阵元最终的发射信号（实数）
%   delays_samp   : [Nt x Na]   各阵元对各子带的整数延时（样点，>=0）
%   excitation_ori : [Mout x 6] 6路信号正交信号
% 说明
% - 远场延时：tau(j,a) = dot(TX(j,:), s_a)/c，其中 s_a=[sinθ,0,cosθ]。
% - 子带LFM定义：
%     up  : f(t) 从 (fc-BW/2) → (fc+BW/2)
%     down: f(t) 从 (fc+BW/2) → (fc-BW/2)
% - 所有子带序列在各自延时后叠加到对应阵元通道。
%
% 复杂度：O(Nt*Na + sum_a pw(a)*fs)，对典型 Nt,Na 规模直接可用。

    arguments
        TX double {mustBeFinite}
        angles_deg double {mustBeFinite, mustBeVector}
        fc double {mustBeFinite, mustBeVector}
        BW double {mustBeFinite, mustBeVector, mustBeNonnegative}
        pol
        pw double {mustBeFinite, mustBeVector, mustBePositive}
        fs double {mustBeFinite, mustBePositive}
        c  double {mustBeFinite, mustBePositive}
        opts.round char {mustBeMember(opts.round,{'round','floor','ceil'})} = 'round'
        opts.win   char {mustBeMember(opts.win,  {'none','hamming'})} = 'hamming'
        opts.amp   double = []
        opts.pad   double {mustBeNonnegative} = 0
    end
    fs = 10*fs;
    TX = double(TX);
    [Nt, dim] = size(TX);
    assert(dim==3, 'TX must be Nt×3');
    arrayWin = hamming(Nt);
    angles_deg = angles_deg(:);  fc = fc(:);  BW = BW(:);  pw = pw(:);
    Na = numel(angles_deg);
    assert(numel(fc)==Na && numel(BW)==Na && numel(pw)==Na, ...
        'angles, fc, BW, pw 长度必须一致');

    % 规范化极性
    if isstring(pol) || ischar(pol)
        pol = cellstr(pol);
    end
    pol = string(pol(:));
    assert(numel(pol)==Na, 'pol 长度必须和 Na 相同');
    ok = pol=="up" | pol=="down";
    assert(all(ok), 'pol 仅支持 "up" 或 "down"');

    % 子带幅度
    if isempty(opts.amp)
        amp = ones(Na,1);
    else
        amp = opts.amp(:);
        assert(numel(amp)==Na, 'opts.amp 长度必须为 Na');
    end

    % --------- 计算远场单位指向向量 s_a（仅方位角；俯仰=0） ----------
    th = deg2rad(angles_deg);               % [Na x 1]
    S  = [sin(th), zeros(Na,1), cos(th)];   % [Na x 3], 每行 s_a

    % --------- 计算延时（秒 & 样点） -----------------------------------
    % tau(j,a) = dot(TX(j,:), S(a,:))/c
    tau = (TX * S.')./c;                    % [Nt x Na]
    switch opts.round
        case 'round', delays_samp = round(tau * fs);
        case 'floor', delays_samp = floor(tau * fs);
        case 'ceil',  delays_samp = ceil(tau * fs);
    end
    % 统一“非负移位”实现：整体右移，使最小延时为0
    dmin = min(delays_samp(:));
    if dmin < 0
        delays_samp = delays_samp - dmin;   % 让所有延时 >= 0
    end

    % --------- 为每个子带生成自身 LFM 波形（实信号） -------------------
    % 定义：f0->f1, T=pw(a)，使用 chirp 等效生成；可选加窗
    sigs = cell(Na,1);          % 每个子带的 1×Na cell
    Ls   = zeros(Na,1);         % 每个子带长度（样点）
    Ma = max([1;round(pw*fs)]);
    for a = 1:Na
        xa = zeros(1,Ma);
        Mpwa = max(1, round(pw(a)*fs));
        t  = (0:Mpwa-1)/fs;
        f0 = fc(a) - BW(a)/2;
        f1 = fc(a) + BW(a)/2;
        if pol(a)=="down"
            [f0, f1] = deal(f1, f0);   % 下扫：高→低
        end

        % 线性调频相位：phi(t) = 2π ( f0 t + 0.5 k t^2 )
        % k   = (f1 - f0)/pw(a);         % 频率斜率 (Hz/s)
        % phi = 2*pi*( f0*t + 0.5*k*t.^2 );
        % 
        % xa(1:Mpwa)  = cos(phi);
        xa(1:Mpwa)  = chirp(t,f0,pw(a),f1);
        % 可选窗
        switch opts.win
            case 'hamming'
                xa = xa .* hamming(Ma).';
            case 'none'
                % do nothing
        end

        % 子带幅度
        xa = amp(a) * xa;

        sigs{a} = xa(:);               % 列向量
        excitation_ori(:,a) = resample(sigs{a},1,10);
        Ls(a)   = Ma;
    end

    % --------- 分配输出总长度：考虑所有子带最大延时 & 额外 pad ----------
    maxDelay = max(delays_samp, [], 'all');           % (样点)
    Mout = max(Ls + maxDelay) + round(opts.pad);      % 输出长度
    excitation_nt = zeros(Nt, Mout, 'double');

    % --------- 把每个子带按各阵元延时“整点叠加”到 excitation_nt ----------
    for a = 1:Na
        xa = sigs{a};       % [La x 1]
        La = numel(xa);
        for j = 1:Nt
            d = delays_samp(j,a);     % 非负整数样点
            idx = (1:La) + d;         % 放置区间
            % 安全截断（通常不需，Mout 已保证足够大）
            idx_valid = idx(idx<=Mout);
            La_valid  = numel(idx_valid);
            if La_valid>0
                excitation_nt(j, idx_valid) = excitation_nt(j, idx_valid) + xa(1:La_valid).';
            end
        end
    end
    for j = 1:Nt
        excitation_nt(j,:) = excitation_nt(j,:)*arrayWin(j);
    end
    excitation_nt = resample(excitation_nt',1,10)';
end
