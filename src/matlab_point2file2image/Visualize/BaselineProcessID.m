% clear all;close all;clc
addpath("Lib\");
% 基本用法：读取所有数据
data = ReadBaselineHDF5('./BaselineData/default.h5');
win = hamming(data.attributes.receive_array_num);
cfg.rx_xyz = data.attributes.receive_array_position;
cfg.c = data.attributes.sound_velocity;
cfg.t0 = data.attributes.sample_delay-data.attributes.pulse_duration/2;                      % 用你的实际 t_start
cfg.fs = data.attributes.sampling_frequency/data.attributes.decimate_factor;
cfg.angles = data.attributes.scan_angle * pi/180;   % 扫描角（rad）
cfg.BW = data.attributes.bandwidth;
cfg.f0 = data.attributes.center_frequency;
cfg.fd.nfft   =  2^nextpow2(length(data.pings{1}(:,1)));          % 或 64 / 256，越大分数时延越精细，但更慢
cfg.fd.sign   = 1;
cfg.cf.enable = false;
cfg.cf.mode = 'cf';
cfg.cf.gamma = 0.05;
% for i = 1:data.attributes.ping_num
for i = 1:1
    ping_data = data.pings{i};
    
    %% Match Filter

    
    sig_mf = zeros(size(ping_data));
    for m = 1:data.attributes.receive_array_num
        tmp = conv(ping_data(:,m), data.attributes.match_filter_data, 'same');
        sig_mf(:,m) = tmp*win(m);
    end
    clear tmp ping_data m;
    
    %% TVG
    % To-do
    
    %% Beamforming
    [beam,ranges_out] = das_plane_wave_id(cfg, sig_mf);
    % figure,
    sector_plot(cfg.angles,ranges_out, abs(beam), 40,"sector");
end
