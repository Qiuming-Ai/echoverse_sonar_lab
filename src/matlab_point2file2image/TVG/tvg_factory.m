function tvg = tvg_factory(t_start, varargin)
% TVG_FACTORY  构造“时间可变增益”补偿函数 G(τ)
%  输入
%    t_start : 开始记录时刻（相对发射的两程时间, s）
%  可选参数（Name-Value）：
%    'c'            : 声速 (m/s), 默认 1500
%    'mode'         : 'point' (K=40) | 'volume' (K=20) | 'intermediate' (K=30) | 'custom'
%    'K'            : 自定义对数斜率, 仅当 mode='custom' 有效
%    'alpha_dB_m'   : 吸收 (dB/m). 若未提供且给出 'freq_kHz' 则用 Thorp 计算
%    'freq_kHz'     : 频率 (kHz)，用于 Thorp 计算吸收
%    'R0'           : 参考距离 (m), 默认 1
%    'G0_dB'        : 额外常数增益 (dB), 默认 0
%    'clamp_Rmin'   : 距离下限，避免 log(0)，默认 0.1 m
%
%  输出
%    tvg : 结构体，包含函数句柄
%          tvg.GdB(τ)   -> dB增益
%          tvg.Gamp(τ)  -> 幅度域增益因子（乘到包络/幅度上）
%          tvg.Gpow(τ)  -> 功率域增益因子（乘到功率/强度上）

p = inputParser;
addParameter(p,'c',1500);
addParameter(p,'mode','volume'); % 'point'|'volume'|'intermediate'|'custom'
addParameter(p,'K',40);          % 仅 mode='custom' 时使用
addParameter(p,'alpha_dB_m',[]);
addParameter(p,'freq_kHz',[]);
addParameter(p,'R0',1);
addParameter(p,'G0_dB',0);
addParameter(p,'clamp_Rmin',0.1);
parse(p,varargin{:});
c          = p.Results.c;
mode       = lower(p.Results.mode);
K_custom   = p.Results.K;
alpha_dB_m = p.Results.alpha_dB_m;
freq_kHz   = p.Results.freq_kHz;
R0         = p.Results.R0;
G0_dB      = p.Results.G0_dB;
Rmin       = p.Results.clamp_Rmin;

% 选择 K（两程几何+统计补偿斜率）
switch mode
    case 'point',        K = 40;   % 点目标：40·log10 R
    case 'volume',       K = 20;   % 体散射：20·log10 R
    case 'intermediate', K = 30;   % 常见中间档
    case 'custom',       K = K_custom;
    otherwise, error('mode must be point|volume|intermediate|custom');
end

% 吸收 α(dB/m)
if isempty(alpha_dB_m)
    if isempty(freq_kHz)
        error('未提供 alpha_dB_m 或 freq_kHz（用于Thorp计算）。');
    end
    alpha_dB_m = thorp_alpha_dB_per_m(freq_kHz);
end

% 返回函数句柄：τ -> t_true = t_start + τ
tvg.GdB  = @(tau) tvg_dB_core(t_start + tau, c, K, alpha_dB_m, R0, G0_dB, Rmin);
tvg.Gamp = @(tau) 10.^( tvg.GdB(tau) / 20 ); % 幅度域乘因子
tvg.Gpow = @(tau) 10.^( tvg.GdB(tau) / 10 ); % 功率域乘因子
end

function GdB = tvg_dB_core(t, c, K, alpha_dB_m, R0, G0_dB, Rmin)
% 两程距离
R = max(Rmin, c .* t / 2);
% TVG(dB) = K*log10(R/R0) + 2*alpha*R + G0
GdB = K .* log10(R./R0) + 2*alpha_dB_m.*R + G0_dB;
end

function a_db_m = thorp_alpha_dB_per_m(f_kHz)
% Thorp 经验式, f 以 kHz 计
f2 = f_kHz.^2;
a_db_km = 0.11 .* f2 ./ (1 + f2) ...
        + 44   .* f2 ./ (4100 + f2) ...
        + 2.75e-4 .* f2 + 0.003;
a_db_m = a_db_km * 1e-3;
end
