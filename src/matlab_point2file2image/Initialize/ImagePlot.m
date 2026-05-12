function ImagePlot(sonar, echo)
figure;
beam = [];
for i = 1:sonar.sector_num
    %% Match Filter
    sig_mf = zeros(size(echo.y_deci));
    for m = 1:sonar.Nrx
        tmp = conv(echo.y_deci(:,m), sonar.MF_deci, 'same');
        sig_mf(:,m) = tmp*echo.win(m);
    end
    % TVG
    tvg = tvg_factory(echo.t,'c',sonar.c0,'mode','custom','freq_kHz',sonar.fc/1e3,'G0_dB',0,'K',20);
    % 假设采样率 fs=500 kHz，记录 100 ms：
    tau = (0:length(sig_mf)-1)/echo.fs_deci;  % τ轴
    G_amp = tvg.Gamp(tau)';  % 用于幅度/包络补偿
    for m = 1:sonar.Nrx
        sig_mf(:,m) = sig_mf(:,m).*G_amp;
    end
    %%
    data = sig_mf./max(abs(sig_mf(:)));
    rx = data;
    cfg = struct;
    cfg.rx_xyz = sonar.rx_xyz;
    cfg.c = sonar.c0;
    cfg.t0 = 0-sonar.pulse_len/2;                      % 用你的实际 t_start
    cfg.fs = echo.fs_deci;
    cfg.angles = sonar.angles_div{i} * pi/180;   % 扫描角（rad）
    cfg.BW = sonar.SubBW(i);
    cfg.f0 = sonar.Subfc(i);
    cfg.fd.nfft   = 8196;          % 或 64 / 256，越大分数时延越精细，但更慢
    cfg.fd.sign   = 1;
    cfg.cf.enable = false;
    cfg.cf.mode = 'cf';
    cfg.cf.gamma = 0.2;
    [beam{i},ranges_out] = das_plane_wave_id(cfg, rx);
end
beams = [];
for i = 1:sonar.sector_num
    beams = [beams,beam{i}];
end
%% 
% beams = compensate_range_shift(beams, 1/((cfg.fd.nfft/cfg.fs)/sonar.c0), sonar.c0, sonar.compensate_range(1,:),'true');
cfg.angles = sonar.angles_div{:}/180*pi;
sector_plot(cfg.angles, ranges_out, beams, 40,"sector");
hold on;
end