function file2image(filePath)
if nargin < 1 || strlength(string(filePath)) == 0
    filePath = "./Sonar Data/default.h5";
end
addpath(genpath("./Initialize"))
EnvInit;
% 基本用法：读取所有数据
data = ReadBaselineHDF5(filePath);
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

[outDir, outName, ~] = fileparts(char(filePath));
gifPath = fullfile(outDir, [outName, '.gif']);
if exist(gifPath, 'file')
    delete(gifPath);
end

fig = figure('Visible','off','Color','w');
ax = axes(fig);

for i = 1:data.attributes.ping_num
% for i = 1:1
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
    tvg = tvg_factory(0,'c',cfg.c,'mode','custom','freq_kHz',cfg.f0/1e3,'G0_dB',0,'K',20);
    % 假设采样率 fs=500 kHz，记录 100 ms：
    tau = (0:length(sig_mf)-1)/cfg.fs;  % τ轴
    G_amp = tvg.Gamp(tau)';  % 用于幅度/包络补偿
    for m = 1:data.attributes.receive_array_num
        sig_mf(:,m) = sig_mf(:,m).*G_amp;
    end
    %% Beamforming
    [beam,ranges_out] = das_plane_wave_id(cfg, sig_mf);
    plotData = sector_plot(cfg.angles, ranges_out, abs(beam), 40, "sector", false);

    cla(ax);
    surf(ax, plotData.X, plotData.Z, plotData.IdB, 'EdgeColor', 'none');
    view(ax, 2);
    set(ax, 'YDir', 'reverse');
    colormap(ax, "jet");
    caxis(ax, [-plotData.dynRange 0]);
    title(ax, sprintf('DAS Sector Image - Ping %d', i));
    xlabel(ax, 'Lateral X (m)');
    ylabel(ax, 'Range Z (m)');
    drawnow;

    frame = getframe(fig);
    [im, map] = rgb2ind(frame2im(frame), 256);
    if i == 1
        imwrite(im, map, gifPath, 'gif', 'LoopCount', inf, 'DelayTime', 0.15);
    else
        imwrite(im, map, gifPath, 'gif', 'WriteMode', 'append', 'DelayTime', 0.15);
    end
end
close(fig);
end