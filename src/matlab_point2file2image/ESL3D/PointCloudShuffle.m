function point_position_out = PointCloudShuffle(point_position, wavelength, shuffleRatio, n)
%POINTCLOUDSHUFFLE 随机选取部分点并在 xyz 方向做有限随机扰动。
%
%   point_position_out = PointCloudShuffle(point_position, wavelength, shuffleRatio, n)
%
% 输入:
%   point_position : N×3 点云坐标
%   wavelength     : 波长 (标量, >0)
%   shuffleRatio   : 打乱比例 (0~1)
%   n              : 扰动倍数 (>=0)
%
% 规则:
%   - 随机选择 round(N * shuffleRatio) 个点
%   - 对被选中的每个点, 每个坐标轴独立扰动:
%       delta in [-n*wavelength, +n*wavelength]
%   - 未选中的点保持不变
%
% 输出:
%   point_position_out : 扰动后的 N×3 点云坐标

    arguments
        point_position (:, 3) {mustBeNumeric, mustBeFinite}
        wavelength (1, 1) double {mustBeNumeric, mustBeFinite, mustBePositive}
        shuffleRatio (1, 1) double {mustBeNumeric, mustBeFinite}
        n (1, 1) double {mustBeNumeric, mustBeFinite, mustBeNonnegative}
    end

    if shuffleRatio < 0 || shuffleRatio > 1
        error("PointCloudShuffle:InvalidShuffleRatio", ...
            "shuffleRatio 必须在区间 [0, 1] 内。");
    end

    point_position_out = point_position;
    numPoints = size(point_position, 1);
    if numPoints == 0 || shuffleRatio == 0 || n == 0
        return;
    end

    numShuffle = round(numPoints * shuffleRatio);
    numShuffle = max(0, min(numPoints, numShuffle));
    if numShuffle == 0
        return;
    end

    idx = randperm(numPoints, numShuffle);
    maxDelta = n * wavelength;

    % 每个轴独立均匀随机扰动, 范围 [-maxDelta, +maxDelta]
    delta = (2 * rand(numShuffle, 3) - 1) * maxDelta;
    point_position_out(idx, :) = point_position_out(idx, :) + cast(delta, class(point_position_out));
end
