function [excitation_nt, excitation_ori, delays_samp] = gen_multicode_cazac_excitation( ...
        TX, angles_deg, fc, BW, fs, c, opts)
% gen_multicode_cazac_excitation
% 基于多路 CAZAC（Zadoff-Chu）码的远场整数延时聚焦 → 每个阵元的发射时域序列。
%
% 输入
%   TX         : [Nt x 3] 阵元坐标 (m)
%   angles_deg : [Na x 1] 各路信号对应的指向方位角（度）；0°=+z方向，+θ向+x偏
%   fc         : 标量，各路公共中心频率 (Hz)
%   BW         : 标量，公共占用带宽 (Hz)
%   fs         : 标量，最终输出采样率 (Hz)
%   c          : 标量，声速 (m/s)
%   opts       : 结构体，可选字段：
%                - round : 'round'(默认) | 'floor' | 'ceil'  延时取整方式
%                - pad   : 额外尾部零填充样点数（默认 0）
%                - amp   : [Na x 1] 各路幅度系数（默认全 1）
%                - roots : [1 x Na] CAZAC 根（默认 {1,3,5,7,9,11}）
%                - N     : 标量，CAZAC 长度（默认 1021，建议为质数）
%                - win   : 'none'(默认) | 'hann' | 'hamming' | 'blackman' | 'kaiser6'
%                          作用于时域 CAZAC 符号的幅度窗
%                - realOutput : logical，true(默认) 表示输出实数载波信号，
%                                false 输出复基带信号
%
% 输出
%   excitation_nt : [Nt x Mout] 各阵元最终发射信号
%   excitation_ori: [Mmax x Na] 各路原始（未延时）信号，按列零填充齐次
%   delays_samp   : [Nt x Na]   各阵元对各路信号的整数延时（样点，>=0）

    arguments
        TX double {mustBeFinite}
        angles_deg double {mustBeFinite, mustBeVector}
        fc double {mustBeFinite, mustBeScalarOrEmpty, mustBePositive}
        BW double {mustBeFinite, mustBeScalarOrEmpty, mustBeNonnegative}
        fs double {mustBeFinite, mustBePositive}
        c  double {mustBeFinite, mustBePositive}
        opts.round char {mustBeMember(opts.round,{'round','floor','ceil'})} = 'round'
        opts.pad double {mustBeNonnegative} = 0
        opts.amp double = []
        opts.roots double = [5,7,1,3,11,9]
        opts.N double {mustBePositive, mustBeInteger} = 513
        opts.win char {mustBeMember(opts.win,{'none','hann','hamming','blackman','kaiser6'})} = 'none'
        opts.realOutput logical = true
    end

    TX = double(TX);
    [Nt, dim] = size(TX);
    assert(dim==3, 'TX must be Nt×3');

    angles_deg = angles_deg(:);
    roots = opts.roots(:).';
    Na_roots = numel(roots);
    if isempty(angles_deg)
        error('angles_deg 不能为空');
    end
    if numel(angles_deg)==1 && Na_roots>1
        angles_deg = repmat(angles_deg, Na_roots, 1);
    end
    if numel(angles_deg) ~= Na_roots
        error('angles_deg 与 opts.roots 长度必须一致，或 angles_deg 为标量自动广播。');
    end
    Na = Na_roots;

    if isempty(opts.amp)
        amp = ones(Na,1);
    else
        amp = opts.amp(:);
        assert(numel(amp)==Na, 'opts.amp 长度必须为 Na');
    end

    N = opts.N;
    assert(N>0 && mod(N,1)==0, 'opts.N 必须为正整数');
    if mod(N,2)==0
        warning('建议 opts.N 取质数或奇数以获得良好的 CAZAC 相关特性。');
    end

    % --------- 计算远场单位指向向量 s_a（仅方位角；俯仰=0） ----------
    th = deg2rad(angles_deg);               % [Na x 1]
    S  = [sin(th), zeros(Na,1), cos(th)];   % [Na x 3]

    % --------- 计算延时（秒 & 样点） -----------------------------------
    tau = (TX * S.')./c;                    % [Nt x Na]
    switch opts.round
        case 'round', delays_samp = round(tau * fs);
        case 'floor', delays_samp = floor(tau * fs);
        case 'ceil',  delays_samp = ceil(tau * fs);
    end
    dmin = min(delays_samp(:));
    if dmin < 0
        delays_samp = delays_samp - dmin;
    end

    % --------- 生成未延时的 CAZAC 波形（每路） -------------------------
    fs_symbol = BW;
    if fs_symbol <= 0
        error('BW 必须为正数。');
    end
    n = (0:N-1).';

    sigs = cell(Na,1);
    sigLen = zeros(Na,1);
    for a = 1:Na
        r = roots(a);
        X = exp(-1j*pi*r*n.*(n+1)/N); % 频域等幅、相位按 CAZAC 根
        x = ifft(X);                  % 基带 CAZAC 序列
        x = x / sqrt(mean(abs(x).^2));% 单位功率

        % 施加时域窗
        win = localWindow(opts.win, N);
        x = x .* win;

        % 将基带序列视为 BW 采样率，重采样到目标 fs
        if abs(fs - fs_symbol) < eps(fs_symbol)
            x_fs = x;
        else
            x_fs = resample(x, fs, fs_symbol);
        end

        t = (0:numel(x_fs)-1).' / fs;
        if opts.realOutput
            sig = real(x_fs .* exp(1j*2*pi*fc*t));
        else
            sig = x_fs .* exp(1j*2*pi*fc*t);
        end

        sig = amp(a) * sig(:);
        sigs{a} = sig;
        sigLen(a) = numel(sig);
    end

    maxLen = max(sigLen);
    excitation_ori = zeros(maxLen, Na);
    for a = 1:Na
        excitation_ori(1:sigLen(a),a) = sigs{a};
    end

    % --------- 叠加至各阵元通道 ---------------------------------------
    maxDelay = max(delays_samp, [], 'all');
    Mout = max(sigLen + maxDelay) + round(opts.pad);
    excitation_nt = zeros(Nt, Mout);

    for a = 1:Na
        xa = sigs{a};       % [La x 1]
        La = numel(xa);
        for j = 1:Nt
            d = delays_samp(j,a);
            idx = (1:La) + d;
            idx_valid = idx(idx<=Mout);
            La_valid  = numel(idx_valid);
            if La_valid>0
                excitation_nt(j, idx_valid) = excitation_nt(j, idx_valid) + xa(1:La_valid).';
            end
        end
    end

    % 阵元加权（保持与 LFM 版本一致，这里使用汉明窗）
    arrayWin = hamming(Nt);
    for j = 1:Nt
        excitation_nt(j,:) = excitation_nt(j,:) * arrayWin(j);
    end
end

% -------------------------------------------------------------------------
function win = localWindow(type, N)
    switch type
        case 'none'
            win = ones(N,1);
        case 'hann'
            win = hann(N,'periodic');
        case 'hamming'
            win = hamming(N,'periodic');
        case 'blackman'
            win = blackman(N,'periodic');
        case 'kaiser6'
            beta = 6;
            win = kaiser(N,beta);
        otherwise
            error('未知窗类型: %s', type);
    end
    win = win(:);
end

