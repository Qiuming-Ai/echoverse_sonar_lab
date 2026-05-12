function pointCloudData = SceneInit(esl3dPath)
if nargin < 1 || strlength(string(esl3dPath)) == 0
    esl3dPath = "./Point Cloud/default.esl3d";
end
esl3dPath = char(strtrim(string(esl3dPath)));
esl3dPath = strrep(esl3dPath, '/', filesep);

if ~isfile(esl3dPath)
    error('SceneInit:FileNotFound', 'esl3d file not found: %s', esl3dPath);
end

[~, ~, ext] = fileparts(esl3dPath);
if ~strcmpi(ext, '.esl3d')
    error('SceneInit:InvalidExtension', 'Expected .esl3d file, got: %s', esl3dPath);
end

pointCloudData = esl3d(esl3dPath);
end
% [point_position,point_amplitudes] = pointCloudData.getPointCloud(1);
% % point_position = PointCloudShuffle(point_position, lambda, 0.9, 3);
% [point_position,point_amplitudes] = PointCloudDecimate(point_position, point_amplitudes, 0.2);