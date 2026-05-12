function pointcloud2file(sonarparams_path)
if nargin < 1 || strlength(string(sonarparams_path)) == 0
    sonarparams_path = "./SonarParameter/Sonar.json";
end
addpath(genpath("./Initialize"))
EnvInit;
sonar = SonarInit(sonarparams_path);
dataMaker = DataMakerInit(sonar);
pointCloudData = SceneInit(sonar.esl3d_path);
for i = 1:pointCloudData.getFrameCount()
    [point_position,point_amplitudes] = pointCloudData.getPointCloud(i);
    % point_position = PointCloudShuffle(point_position, sonar.lambda, 0.9, 3);
    [point_position,point_amplitudes] = PointCloudDecimate(point_position, point_amplitudes, 0.3);
    echo = EchoInit(sonar, point_position, point_amplitudes);
    dataMaker.write(echo.y_deci);
end
dataMaker.close();
end