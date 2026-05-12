function echo = EchoInit(sonar, point_position, point_amplitudes)
opts = struct('round','round','precision','single');
opts.array.use_resample = true;                   % 若 true 尝试调用 MATLAB resample

use_nvidia_cuda = false;
try
    if gpuDeviceCount > 0
        g = gpuDevice;
        use_nvidia_cuda = contains(g.Name, 'NVIDIA', 'IgnoreCase', true);
    end
catch
    use_nvidia_cuda = false;
end
% use_nvidia_cuda = false;
if use_nvidia_cuda
    [v, t] = sim_rx_from_scatterers_perTX_cuda(point_position,point_amplitudes,sonar.tx_xyz,sonar.rx_xyz,sonar.fc,sonar.c0,sonar.interp_factor*sonar.fs,sonar.exc_nt,opts);
else
    [v, t] = sim_rx_from_scatterers_perTX(point_position,point_amplitudes,sonar.tx_xyz,sonar.rx_xyz,sonar.fc,sonar.c0,sonar.interp_factor*sonar.fs,sonar.exc_nt,opts);
end
% Pad front zeros according to propagation delay t (seconds).
delay_samples = max(0, round(double(t) * sonar.interp_factor * sonar.fs));
if delay_samples > 0
    v = [zeros(delay_samples, size(v, 2), 'like', v); v];
end
v = (v-min(v(:)))./(max(v(:))-min(v(:)));
v = (v-0.5)*2;

v =  resample(v,1,sonar.interp_factor);

win = eval([sonar.array_window,'(',num2str(sonar.Nrx),')']);

v_calc = resample(v,sonar.c0+2*sonar.velocity,sonar.c0);
v_calc = awgn(v_calc,sonar.snr_level,'measured');
%% 下变频 To do
v_mix = v_calc.*repmat(exp(1j*2*pi*sonar.Subfc(1)*[0:length(v_calc(:,1))-1]/sonar.fs)',[1 sonar.Nrx]);
%% FIR滤波器
y_fir = sonar.filt.process(v_mix);
%% Decimation
y_deci = resample(y_fir,1,sonar.decimation_factor);
fs_deci = sonar.fs/sonar.decimation_factor;

echo = struct();
echo.y_deci = y_deci;
echo.fs_deci = fs_deci;
echo.t = t;
echo.win = win;
echo.v = v;
echo.v_calc = v_calc;
echo.v_mix = v_mix;
echo.y_fir = y_fir;
end