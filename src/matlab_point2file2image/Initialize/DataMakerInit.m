function dataMaker = DataMakerInit(sonar)
% DataMakerInit
% Initialize SonarDataMaker using parameters prepared in SonarInit.m.

if ~isfield(sonar, 'tx_type')
    error('DataMakerInit:MissingVar', ...
        'tx_type is missing. Please run SonarInit first.');
end

if ~isfield(sonar, 'decimation_factor')
    error('DataMakerInit:MissingVar', ...
        'decimation_factor is missing. Please run SonarInit first.');
end

if ~isfield(sonar, 'MF')
    error('DataMakerInit:MissingVar', ...
        'MF is missing. Please run SonarInit first.');
end

if ~isfield(sonar, 'esl3d_path') || strlength(string(sonar.esl3d_path)) == 0
    error('DataMakerInit:MissingVar', ...
        'esl3d_path is missing. Please run SonarInit first.');
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
tx_type_str = lower(string(sonar.tx_type));

switch tx_type_str
    case "cdm"
        file_prefix = 'CDMData';
        array_type = 'CDM';
    case "fdm"
        file_prefix = 'FDMData';
        array_type = 'FDM';
    otherwise
        file_prefix = 'BaselineData';
        array_type = 'Baseline';
end

if isfield(sonar, 'array_window')
    if ischar(sonar.array_window) || (isstring(sonar.array_window) && isscalar(sonar.array_window))
        window_name = char(sonar.array_window);
        switch lower(window_name)
            case 'hamming'
                receive_array_win = hamming(sonar.Nrx);
            case 'hann'
                receive_array_win = hann(sonar.Nrx);
            case 'blackman'
                receive_array_win = blackman(sonar.Nrx);
            otherwise
                receive_array_win = ones(sonar.Nrx, 1);
        end
    else
        receive_array_win = sonar.array_window;
        window_name = 'custom';
    end
else
    receive_array_win = hamming(sonar.Nrx);
    window_name = 'hamming';
end

if isfield(sonar, 'angles_div') && iscell(sonar.angles_div) && ~isempty(sonar.angles_div)
    sector_edges = zeros(1, numel(sonar.angles_div) + 1);
    sector_edges(1) = sonar.angles_div{1}(1);
    for i = 1:numel(sonar.angles_div)
        sector_edges(i + 1) = sonar.angles_div{i}(end);
    end
else
    sector_edges = [];
end

match_filter_data = sonar.MF_deci;

if isfield(sonar, 'Subfc') && numel(sonar.Subfc) > 1
    center_frequency = sonar.Subfc;
elseif isfield(sonar, 'fc')
    center_frequency = sonar.fc;
else
    center_frequency = [];
end

if isfield(sonar, 'SubBW') && numel(sonar.SubBW) > 1
    bandwidth = sonar.SubBW;
elseif isfield(sonar, 'BW')
    bandwidth = sonar.BW;
else
    bandwidth = [];
end

if isfield(sonar, 'sector_num')
    local_sector_num = sonar.sector_num;
else
    local_sector_num = 1;
end

if isfield(sonar, 'angles_div')
    scan_angle = sonar.angles_div{:};
else
    scan_angle = -60:1:60;
end

sonarInfo = struct();
sonarInfo.array_type = array_type;
sonarInfo.signal_type = 'Baseband';
sonarInfo.signal_win = window_name;
sonarInfo.bandwidth = bandwidth;
sonarInfo.sampling_frequency = sonar.fs;
sonarInfo.center_frequency = center_frequency;
sonarInfo.decimate_factor = sonar.decimation_factor;
sonarInfo.sector_num = local_sector_num;
sonarInfo.match_filter_data = match_filter_data;
sonarInfo.receive_array_num = sonar.Nrx;
sonarInfo.receive_array_position = sonar.rx_xyz;
sonarInfo.receive_array_win = receive_array_win;
sonarInfo.pulse_duration = sonar.pulse_len;
sonarInfo.sound_velocity = sonar.c0;
sonarInfo.velocity = sonar.velocity;
sonarInfo.snr_level = sonar.snr_level;
sonarInfo.timestamp = timestamp;
sonarInfo.scan_angle = scan_angle;

if ~isempty(sector_edges)
    sonarInfo.sector_div = sector_edges;
end
if isfield(sonar, 'compensate_range')
    sonarInfo.sample_delay = sonar.compensate_range;
end

[esl3d_dir, esl3d_name, ~] = fileparts(char(sonar.esl3d_path));
if isfield(sonar, 'output_path') && strlength(string(sonar.output_path)) > 0
    output_dir = char(sonar.output_path);
else
    output_dir = esl3d_dir;
end
sonar_h5_filename = fullfile(output_dir, [esl3d_name, '.h5']);
dataMaker = SonarDataMaker();
dataMaker.start(sonar_h5_filename, sonarInfo);
end
