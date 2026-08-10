function pointcloud2file(sonarparams_path)
if nargin < 1 || strlength(string(sonarparams_path)) == 0
    sonarparams_path = "./SonarParameter/Sonar.json";
end
addpath(genpath("./Initialize"))
EnvInit;
sonar = SonarInit(sonarparams_path);
dataMaker = DataMakerInit(sonar);
pointCloudData = SceneInit(sonar.esl3d_path);

% Optional per-ping profiling for paper performance/scale experiments.
% Example: setenv('ESL_MATLAB_PERF_CSV', fullfile(pwd, 'matlab_performance.csv'))
perfCsvPath = string(getenv('ESL_MATLAB_PERF_CSV'));
perfFid = -1;
if strlength(perfCsvPath) > 0
    perfDir = fileparts(perfCsvPath);
    if strlength(perfDir) > 0 && ~isfolder(perfDir)
        mkdir(perfDir);
    end
    [perfFid, perfMessage] = fopen(perfCsvPath, 'w');
    if perfFid < 0
        error('pointcloud2file:PerformanceLogOpenFailed', ...
            'Cannot open MATLAB performance CSV %s: %s', perfCsvPath, perfMessage);
    end
    perfCleanup = onCleanup(@() fclose(perfFid)); %#ok<NASGU>
    fprintf(perfFid, ['frame_index,total_ms,esl3d_read_ms,decimation_ms,' ...
        'echo_simulation_ms,hdf5_write_ms,input_scatterers,retained_scatterers,' ...
        'output_samples,output_channels,backend\n']);
end

for i = 1:pointCloudData.getFrameCount()
    frameTimer = tic;
    stageTimer = tic;
    [point_position,point_amplitudes] = pointCloudData.getPointCloud(i);
    readMs = toc(stageTimer) * 1000;
    inputScatterers = size(point_position, 1);

    % point_position = PointCloudShuffle(point_position, sonar.lambda, 0.9, 3);
    stageTimer = tic;
    [point_position,point_amplitudes] = PointCloudDecimate(point_position, point_amplitudes, 0.3);
    decimationMs = toc(stageTimer) * 1000;
    retainedScatterers = size(point_position, 1);

    stageTimer = tic;
    echo = EchoInit(sonar, point_position, point_amplitudes);
    echoSimulationMs = toc(stageTimer) * 1000;

    stageTimer = tic;
    dataMaker.write(echo.y_deci);
    hdf5WriteMs = toc(stageTimer) * 1000;
    totalMs = toc(frameTimer) * 1000;

    if perfFid >= 0
        fprintf(perfFid, '%d,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%s\n', ...
            i, totalMs, readMs, decimationMs, echoSimulationMs, hdf5WriteMs, ...
            inputScatterers, retainedScatterers, size(echo.y_deci, 1), ...
            size(echo.y_deci, 2), char(echo.backend));
    end
end
dataMaker.close();
end
