function [point_position_out, point_amplitudes_out] = PointCloudDecimate( ...
        point_position, point_amplitudes, keepFraction, rngSeed)
%POINTCLOUDDECIMATE 对点云随机下采样（与 SceneInit + esl3d 输出配套）。
%
%   [pos, amp] = PointCloudDecimate(point_position, point_amplitudes, keepFraction)
%
%   point_position   : N×3，与 esl3d.getPointCloud 的坐标一致
%   point_amplitudes : N×1 或 N×K，与坐标行一一对应
%   keepFraction     : 保留比例，(0, 1]，例如 0.2 表示随机保留约 20% 的点
%                      若为 1，则原样返回
%
%   可选第四个参数：非负整数 rngSeed，用于固定随机序列（可重复实验）。
%                    不传时可写为 []。
%
%   输出与输入的数值类型（single/double）尽量保持一致。

    arguments
        point_position (:, 3) {mustBeNumeric, mustBeFinite}
        point_amplitudes (:, :) {mustBeNumeric}
        keepFraction (1, 1) double {mustBeNumeric, mustBeFinite}
        rngSeed (1, 1) double {mustBeNumeric} = NaN
    end

    n = size(point_position, 1);
    if size(point_amplitudes, 1) ~= n
        error("PointCloudDecimate:SizeMismatch", ...
            "point_amplitudes 行数 (%d) 必须与 point_position 行数 (%d) 一致。", ...
            size(point_amplitudes, 1), n);
    end

    if keepFraction <= 0 || keepFraction > 1
        error("PointCloudDecimate:InvalidKeepFraction", ...
            "keepFraction 必须在区间 (0, 1] 内。");
    end

    rngState = [];
    if ~isempty(rngSeed) && ~isnan(rngSeed)
        if ~(isfinite(rngSeed) && rngSeed >= 0 && floor(rngSeed) == rngSeed)
            error("PointCloudDecimate:InvalidSeed", ...
                "rngSeed 必须为非负整数标量。");
        end
        rngState = rng;
        rng(double(rngSeed), "twister");
    end

    if n == 0
        point_position_out = point_position;
        point_amplitudes_out = point_amplitudes;
        if ~isempty(rngState)
            rng(rngState);
        end
        return;
    end

    if keepFraction >= 1
        point_position_out = point_position;
        point_amplitudes_out = point_amplitudes;
        if ~isempty(rngState)
            rng(rngState);
        end
        return;
    end

    nKeep = round(n * keepFraction);
    nKeep = max(0, min(n, nKeep));

    if nKeep == 0
        point_position_out = zeros(0, 3, class(point_position));
        point_amplitudes_out = zeros(0, size(point_amplitudes, 2), class(point_amplitudes));
        if ~isempty(rngState)
            rng(rngState);
        end
        return;
    end

    idx = randperm(n, nKeep);
    if ~isempty(rngState)
        rng(rngState);
    end

    point_position_out = point_position(idx, :);
    point_amplitudes_out = point_amplitudes(idx, :);
end
