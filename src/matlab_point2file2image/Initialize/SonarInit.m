function sonar = SonarInit(configPath)
if nargin < 1 || strlength(string(configPath)) == 0
    configPath = "./SonarParameter/Sonar.json";
end

cfg = jsondecode(fileread(configPath));
array_params = cfg.array_params;
tx_signal_params = cfg.tx_signal_params;
rx_signal_params = cfg.rx_signal_params;

c0 = array_params.c0;
fs = array_params.fs;
interp_factor = array_params.interp_factor;
fc = array_params.fc;      % Transducer center frequency [Hz]
BW = array_params.BW;
Nrx = array_params.Nrx;    % Number of elements
Ntx = array_params.Ntx;
pulse_len = array_params.pulse_len;
lightPos = array_params.lightPos;
velocity = array_params.velocity;
snr_level = array_params.snr_level;
compensate_range = array_params.compensate_range';
tx_interval_lambda = array_params.tx_interval_lambda;
rx_interval_lambda = array_params.rx_interval_lambda;

%% 激励脉冲
% 子带参数
tx_type = tx_signal_params.tx_type;
sector_num = tx_signal_params.sector_num;
angles_deg = tx_signal_params.angles_deg(:);  % 3 个方位角
Subfc = tx_signal_params.Subfc(:);            % kHz 级别
SubBW = tx_signal_params.SubBW(:);
array_window = rx_signal_params.array_window;
decimation_factor = rx_signal_params.decimation_factor;
% 接收扫描角分区（度），用于 init.m 中每个子扇区波束形成
angles_div = {};
if isfield(rx_signal_params, "angle_segments_deg") && isfield(rx_signal_params, "angle_step_deg")
    stepDeg = rx_signal_params.angle_step_deg;
    if ~isscalar(stepDeg) || ~isnumeric(stepDeg) || ~isfinite(stepDeg) || stepDeg <= 0
        error("rx_signal_params.angle_step_deg must be a positive scalar.");
    end

    segs = rx_signal_params.angle_segments_deg;
    if ~isnumeric(segs) || size(segs, 2) ~= 2
        error("rx_signal_params.angle_segments_deg must be an N-by-2 numeric array.");
    end

    for i = 1:size(segs, 1)
        startDeg = segs(i, 1);
        endDeg = segs(i, 2);
        if endDeg < startDeg
            error("angle_segments_deg row %d has end < start.", i);
        end
        % 用统一步长展开每个区间（包含起止）
        angles_div{i} = startDeg:stepDeg:endDeg;
    end
elseif isfield(rx_signal_params, "angles_div_deg")
    rawAnglesDiv = rx_signal_params.angles_div_deg;
    if iscell(rawAnglesDiv)
        for i = 1:numel(rawAnglesDiv)
            angles_div{i} = rawAnglesDiv{i}(:).';
        end
    elseif isnumeric(rawAnglesDiv)
        % 若各分区长度一致，jsondecode 可能直接给出数值矩阵
        for i = 1:size(rawAnglesDiv, 1)
            angles_div{i} = rawAnglesDiv(i, :);
        end
    else
        error("rx_signal_params.angles_div_deg must be cell or numeric array.");
    end
else
    % 向后兼容：配置文件中缺失时使用旧默认
    angles_div{1} = -60:0.1:-34-0.1;
    angles_div{2} = -34:0.1:-17-0.1;
    angles_div{3} = -17:0.1:0-0.1;
    angles_div{4} = 0:0.1:17-0.1;
    angles_div{5} = 17:0.1:34-0.1;
    angles_div{6} = 34:0.1:60;
end


dt = 1/fs;
lambda                  = c0/fc;           % Wavelength [m]
rx_xyz = [linspace(-(Nrx-1)*lambda/2*rx_interval_lambda,(Nrx-1)*lambda/2*rx_interval_lambda,Nrx)',zeros(Nrx,2)];
tx_xyz = [linspace(-(Ntx-1)*lambda/2*tx_interval_lambda,(Ntx-1)*lambda/2*tx_interval_lambda,Ntx)',zeros(Ntx,2)];
if tx_type == 'cdm'
    [exc_nt,exc_ori, delays_samp] = gen_multicode_cazac_excitation( ...
        tx_xyz, angles_deg, Subfc(1), SubBW(1), interp_factor*fs, c0,'amp',[1.2 1.2 1 1 1.2 1.2],'N',round(pulse_len/(1/SubBW(1))));
    MF = conj(flipud(hilbert(exc_ori)));                  % FIR 形式
elseif tx_type == 'fdm'
    pol = tx_signal_params.pol;
    pw   = repmat(pulse_len,[6 1]);           % 1.5 ms
    [exc_nt,exc_ori, delays_samp] = gen_multisubband_lfm_excitation( ...
        tx_xyz, angles_deg, Subfc, SubBW, pol, pw, interp_factor*fs, c0);
    MF = conj(flipud(hilbert(exc_ori)));                  % FIR 形式
elseif tx_type == 'lfm'
    t_p=0:1/(interp_factor*fs):pulse_len;
    exc_nt = chirp(t_p,fc-BW/2,pulse_len,fc+BW/2).*hamming(numel(t_p))';
    MF = conj(flipud(hilbert(exc_nt(:))));                  % FIR 形式
end
filt = SignalFilter('SampleRate',fs, ...
    'Passband', max(BW), ...
    'StopbandAtten', 60, ...
    'Type', 'lowpass', ...
    'SidebandMode', 'single', ...
    'OutputSameLength', true);
MF = resample(MF,1,interp_factor);
MF_mix = MF.*(exp(1j*2*pi*Subfc*[0:length(MF(:,1))-1]/(fs))');
for i = 1:sector_num
MF_fir(:,i) = filt.process(MF_mix(:,i));
end
% MF_fir = filt.process(MF_mix(:,i));
MF_deci =  resample(MF_fir,1,decimation_factor);

sonar = struct();
sonar.configPath = configPath;
sonar.cfg = cfg;
sonar.array_params = array_params;
sonar.tx_signal_params = tx_signal_params;
sonar.rx_signal_params = rx_signal_params;
sonar.c0 = c0;
sonar.fs = fs;
sonar.interp_factor = interp_factor;
sonar.fc = fc;
sonar.BW = BW;
sonar.Nrx = Nrx;
sonar.Ntx = Ntx;
sonar.pulse_len = pulse_len;
sonar.lightPos = lightPos;
sonar.velocity = velocity;
sonar.snr_level = snr_level;
sonar.compensate_range = compensate_range;
sonar.tx_interval_lambda = tx_interval_lambda;
sonar.rx_interval_lambda = rx_interval_lambda;
sonar.tx_type = tx_type;
sonar.sector_num = sector_num;
sonar.angles_deg = angles_deg;
sonar.Subfc = Subfc;
sonar.SubBW = SubBW;
sonar.array_window = array_window;
sonar.decimation_factor = decimation_factor;
sonar.angles_div = angles_div;
sonar.dt = dt;
sonar.lambda = lambda;
sonar.rx_xyz = rx_xyz;
sonar.tx_xyz = tx_xyz;
sonar.exc_nt = exc_nt;
sonar.MF = MF;
if exist('pol', 'var')
    sonar.pol = pol;
end
if exist('pw', 'var')
    sonar.pw = pw;
end
if exist('t_p', 'var')
    sonar.t_p = t_p;
end
sonar.filt = filt;
sonar.MF_mix = MF_mix;
sonar.MF_fir = MF_fir;
sonar.MF_deci = MF_deci;
if isfield(cfg, 'file_opt_params') && isfield(cfg.file_opt_params, 'esl3d_path')
    sonar.esl3d_path = cfg.file_opt_params.esl3d_path;
end
if isfield(cfg, 'file_opt_params') && isfield(cfg.file_opt_params, 'output_path')
    sonar.output_path = cfg.file_opt_params.output_path;
end

% if ~isfield(sonar, 'esl3d_path')
%     error('pointcloud2file:MissingField', 'sonar.esl3d_path is missing in SonarInit output.');
% end
% sonar.esl3d_path = char(strtrim(string(sonar.esl3d_path)));
% sonar.esl3d_path = strrep(sonar.esl3d_path, '/', filesep);
end