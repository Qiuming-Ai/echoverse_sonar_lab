function output = compensate_range_shift(data, fs, c, comp_dist, circular_shift)
% 根据补偿距离对数据进行距离维度的偏移、裁剪和补零
%
% Inputs:
%   data          : 输入数据，可以是：
%                   - cell类型：包含l个M*x的矩阵，x为角度维度（任意值），M为距离维度
%                   - M*N矩阵：M为距离维度，N为角度维度
%   fs            : 采样率 (Hz)
%   c             : 声速 (m/s)
%   comp_dist     : 补偿距离序列
%                   - 如果data是cell类型：1*l的序列（每个元素对应一个cell）
%                   - 如果data是矩阵：单个数值
%   circular_shift: (可选) 是否使用循环移位，默认为false
%                   - false: 补零+裁剪模式（默认）
%                   - true: 循环移位模式（裁剪部分拼接到另一边）
%
% Outputs:
%   output        : 输出矩阵
%                   - cell输入：M*y矩阵，y=sum(x)，补偿后按顺序拼接
%                   - 矩阵输入：M*N矩阵，同输入维度
%
% 功能说明：
%   1. 根据补偿距离计算对应的点数：n_points = comp_dist / c * fs
%   2. 对距离维度进行偏移（正负几个点）
%   3. 保持原M长度不变：
%      - 补零模式（circular_shift=false）：裁剪加补零
%      - 循环移位模式（circular_shift=true）：裁剪部分拼接到另一边

% 检查输入参数
if nargin < 4
    error('需要至少4个输入参数：data, fs, c, comp_dist');
end

% 设置默认值
if nargin < 5 || isempty(circular_shift)
    circular_shift = false;
end

if isempty(data)
    error('输入数据不能为空');
end

% 判断输入数据类型
is_cell_input = iscell(data);

if is_cell_input
    % ========== Cell类型输入处理 ==========
    l = length(data);
    
    % 检查comp_dist长度
    if length(comp_dist) ~= l
        error('cell输入时，comp_dist应为1*l的序列，长度与cell数量匹配');
    end
    
    % 获取第一个矩阵的M维度（距离维度）
    if isempty(data{1})
        error('cell中不能包含空矩阵');
    end
    M = size(data{1}, 1);
    
    % 检查所有cell的M维度是否一致
    total_angle_dims = 0;
    for i = 1:l
        if size(data{i}, 1) ~= M
            error('cell中所有矩阵的距离维度M必须一致');
        end
        total_angle_dims = total_angle_dims + size(data{i}, 2);
    end
    
    % 预分配输出矩阵：M * (sum of all angle dimensions)
    output = zeros(M, total_angle_dims, 'like', data{1});
    
    % 处理每个cell
    col_start = 1;
    for i = 1:l
        matrix_i = data{i};
        [~, N_i] = size(matrix_i);
        
        % 计算补偿点数
        n_shift = round(comp_dist(i) / (c) * fs);
        
        % 对距离维度进行偏移
        if circular_shift
            % 循环移位模式：裁剪部分拼接到另一边
            if n_shift ~= 0
                % 使用circshift进行循环移位
                % n_shift > 0: 向下移位（前面移到后面）
                % n_shift < 0: 向上移位（后面移到前面）
                shifted = circshift(matrix_i, -n_shift, 1);
            else
                shifted = matrix_i;
            end
        else
            % 补零模式：补零+裁剪
            if n_shift > 0
                % 向下偏移：前面补零，后面裁剪
                shifted = [zeros(n_shift, N_i, 'like', matrix_i); ...
                           matrix_i(1:end-n_shift, :)];
            elseif n_shift < 0
                % 向上偏移：前面裁剪，后面补零
                shifted = [matrix_i(-n_shift+1:end, :); ...
                      zeros(-n_shift, N_i, 'like', matrix_i)];
            else
                % 无偏移
                shifted = matrix_i;
            end
            
            % 确保输出长度为M（裁剪或补零）
            if size(shifted, 1) > M
                shifted = shifted(1:M, :);
            elseif size(shifted, 1) < M
                shifted = [shifted; zeros(M - size(shifted, 1), N_i, 'like', shifted)];
            end
        end
        
        % 将处理后的矩阵放入输出
        col_end = col_start + N_i - 1;
        output(:, col_start:col_end) = shifted;
        col_start = col_end + 1;
    end
    
else
    % ========== 矩阵类型输入处理 ==========
    [M, N] = size(data);
    
    % 检查comp_dist是否为单个数值
    if ~isscalar(comp_dist)
        error('矩阵输入时，comp_dist应为单个数值');
    end
    
    % 计算补偿点数
    n_shift = round(comp_dist / (c) * fs);
    
    % 对距离维度进行偏移
    if circular_shift
        % 循环移位模式：裁剪部分拼接到另一边
        if n_shift ~= 0
            % 使用circshift进行循环移位
            % n_shift > 0: 向下移位（前面移到后面）
            % n_shift < 0: 向上移位（后面移到前面）
            output = circshift(data, -n_shift, 1);
        else
            output = data;
        end
    else
        % 补零模式：补零+裁剪
        if n_shift > 0
            % 向下偏移：前面补零，后面裁剪
            shifted = [zeros(n_shift, N, 'like', data); ...
                       data(1:end-n_shift, :)];
        elseif n_shift < 0
            % 向上偏移：前面裁剪，后面补零
            shifted = [data(-n_shift+1:end, :); ...
                      zeros(-n_shift, N, 'like', data)];
        else
            % 无偏移
            shifted = data;
        end
        
        % 确保输出长度为M（裁剪或补零）
        if size(shifted, 1) > M
            shifted = shifted(1:M, :);
        elseif size(shifted, 1) < M
            shifted = [shifted; zeros(M - size(shifted, 1), N, 'like', shifted)];
        end
        
        output = shifted;
    end
end

end

