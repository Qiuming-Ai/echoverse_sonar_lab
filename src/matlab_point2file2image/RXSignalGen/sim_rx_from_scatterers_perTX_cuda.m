function [y, t0_ref] = sim_rx_from_scatterers_perTX_cuda(P, A, TX, RX, f0, c, fs, excitation, opts)
% sim_rx_from_scatterers_perTX_cuda
% CUDA加速版本的超声回波合成（Born 近似，单次散射），支持每个发射阵元不同激励。
%
% 输入
%   P   : [K x 3]  散射体坐标 (m)
%   A   : [K x 1]  散射体幅值/散射系数（线性幅度）
%   TX  : [Nt x 3] 发射阵元坐标 (m)（默认"同时触发"，但信号可不同）
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
% 注意：需要先编译CUDA MEX文件：
%   方法1（推荐）：mexcuda -R2018a NVCCFLAGS='-allow-unsupported-compiler' sim_rx_from_scatterers_perTX_cuda_mex.cu
%   方法2：setenv('NVCCFLAGS', '-allow-unsupported-compiler'); mexcuda -R2018a sim_rx_from_scatterers_perTX_cuda_mex.cu

    if nargin < 9 || ~isfield(opts,'round'),   opts.round = 'round'; end
    if ~isfield(opts,'precision'),             opts.precision = 'single'; end
    if ~isfield(opts,'atten'),                 opts.atten = 'none'; end
    if ~isfield(opts,'delay'),                 opts.delay = 0; end

    % 统一类型
    P  = double(P);   A  = double(A(:));
    TX = double(TX);  RX = double(RX);
    fs_orig = double(fs);
    fs = fs_orig * 10;  % 上采样10倍
    c  = double(c);
    
    % 上采样excitation
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

    % 确定round模式
    round_mode = 0;  % 0=round, 1=floor, 2=ceil
    switch lower(opts.round)
        case 'round', round_mode = 0;
        case 'floor', round_mode = 1;
        case 'ceil',  round_mode = 2;
        otherwise, error('opts.round 仅支持 round/floor/ceil');
    end

    % 确定衰减模式
    atten_mode = 0;  % 0=none, 1=twoway_R, 2=sqrt_twoway_R
    switch lower(opts.atten)
        case 'none',           atten_mode = 0;
        case 'twoway_r',       atten_mode = 1;
        case 'sqrt_twoway_r',  atten_mode = 2;
        otherwise
            error('opts.atten 仅支持 none / twoway_r / sqrt_twoway_r');
    end

    % 确定精度
    is_single = strcmpi(opts.precision, 'single');
    
    % 调用CUDA MEX函数
    % 注意：需要先编译 sim_rx_from_scatterers_perTX_cuda_mex.cu
    % 编译命令：mexcuda -R2018a sim_rx_from_scatterers_perTX_cuda_mex.cu
    try
        [y, t0_ref] = sim_rx_from_scatterers_perTX_cuda_mex(...
            P, A, TX, RX, c, fs, excitation, ...
            round_mode, atten_mode, delayK, double(is_single));
    catch ME
        if contains(ME.message, 'sim_rx_from_scatterers_perTX_cuda_mex')
            error(['MEX文件未找到。请先编译CUDA文件：\n' ...
                   '  mexcuda -R2018a sim_rx_from_scatterers_perTX_cuda_mex.cu']);
        else
            rethrow(ME);
        end
    end
    
    % 下采样回原始采样率
    y = resample(y, 1, 10);
end

