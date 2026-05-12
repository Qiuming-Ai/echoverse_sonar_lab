function data = ReadBaselineHDF5(filename, varargin)
% ReadBaselineHDF5 - 读取 BaselineData HDF5 文件的通用函数
%
% 用法:
%   data = ReadBaselineHDF5(filename)
%   data = ReadBaselineHDF5(filename, 'GroupName', '/raw_data')
%   data = ReadBaselineHDF5(filename, 'ReadPings', true, 'ReadAttributes', true)
%
% 输入参数:
%   filename        - HDF5 文件名（字符串）
%   'GroupName'     - 要读取的组名（默认: '/raw_data'）
%   'ReadPings'     - 是否读取 ping 数据（默认: true）
%   'ReadAttributes'- 是否读取 attributes（默认: true）
%   'PingIndices'   - 要读取的 ping 索引（默认: 全部）
%   'Verbose'       - 是否显示详细信息（默认: false）
%
% 输出:
%   data            - 结构体，包含以下字段:
%       .pings          - 元胞数组，每个元素是一个 ping 的数据 [channels x samples]
%       .attributes     - 结构体，包含所有 attributes
%       .info           - 文件信息结构体
%
% 示例:
%   % 读取所有数据
%   data = ReadBaselineHDF5('BaselineData_20240101_120000.h5');
%
%   % 只读取前 5 个 ping
%   data = ReadBaselineHDF5('BaselineData_20240101_120000.h5', 'PingIndices', 1:5);
%
%   % 只读取 attributes
%   data = ReadBaselineHDF5('BaselineHDF5', 'ReadPings', false);

    % 解析输入参数
    p = inputParser;
    addParameter(p, 'GroupName', '/raw_data', @ischar);
    addParameter(p, 'ReadPings', true, @islogical);
    addParameter(p, 'ReadAttributes', true, @islogical);
    addParameter(p, 'PingIndices', [], @isnumeric);
    addParameter(p, 'Verbose', false, @islogical);
    parse(p, varargin{:});
    
    group_name = p.Results.GroupName;
    read_pings = p.Results.ReadPings;
    read_attributes = p.Results.ReadAttributes;
    ping_indices = p.Results.PingIndices;
    verbose = p.Results.Verbose;
    
    % 检查文件是否存在
    if ~exist(filename, 'file')
        error('文件不存在: %s', filename);
    end
    
    % 初始化输出结构
    data = struct();
    data.pings = {};
    data.attributes = struct();
    data.info = struct();
    data.info.filename = filename;
    data.info.group_name = group_name;
    
    % 获取文件信息
    try
        info = h5info(filename);
        if isfield(info, 'FileSize')
            data.info.file_size = info.FileSize;
        end
        if isfield(info, 'Groups') && ~isempty(info.Groups)
            data.info.groups = {info.Groups.Name};
        else
            data.info.groups = {};
        end
    catch ME
        if verbose
            warning('无法读取文件信息: %s', E.message);
        end
        data.info.file_size = [];
        data.info.groups = {};
    end
    
    % 读取 attributes
    if read_attributes
        attrs_group = [group_name '/.attributes'];
        try
            % 检查 .attributes 组是否存在
            attrs_info = h5info(filename, attrs_group);
            data.attributes = ReadAttributes(filename, attrs_group, verbose);
        catch ME
            if verbose
                warning('无法读取 attributes: %s', E.message);
            end
        end
    end
    
    % 读取 ping 数据
    if read_pings
        try
            group_info = h5info(filename, group_name);
            
            % 查找所有 ping（可能是数据集或组）
            ping_list = {};
            
            % 首先检查数据集（实数 ping 数据）
            if isfield(group_info, 'Datasets')
                for i = 1:length(group_info.Datasets)
                    ds_name = group_info.Datasets(i).Name;
                    % 检查是否是 ping 数据集（ping_1, ping_2, ...）
                    if length(ds_name) >= 6 && strcmp(ds_name(1:5), 'ping_')
                        ping_num = str2double(ds_name(6:end));
                        if ~isnan(ping_num)
                            ping_list{end+1} = struct('name', ds_name, 'number', ping_num, 'type', 'dataset');
                        end
                    end
                end
            end
            
            % 然后检查组（复数 ping 数据，包含 real 和 imag）
            if isfield(group_info, 'Groups')
                for i = 1:length(group_info.Groups)
                    grp_name = group_info.Groups(i).Name;
                    % 提取组名（去掉 group_name 前缀）
                    if length(grp_name) > length(group_name) + 1
                        grp_short_name = grp_name(length(group_name)+2:end);
                    else
                        grp_short_name = grp_name;
                    end
                    
                    % 检查是否是 ping 组（ping_1, ping_2, ...）
                    if length(grp_short_name) >= 6 && strcmp(grp_short_name(1:5), 'ping_')
                        ping_num = str2double(grp_short_name(6:end));
                        if ~isnan(ping_num)
                            % 检查是否已存在（避免重复）
                            exists = false;
                            for j = 1:length(ping_list)
                                if ping_list{j}.number == ping_num
                                    exists = true;
                                    break;
                                end
                            end
                            if ~exists
                                ping_list{end+1} = struct('name', grp_short_name, 'number', ping_num, 'type', 'group');
                            end
                        end
                    end
                end
            end
            
            % 按 ping 编号排序
            if ~isempty(ping_list)
                % 提取所有 ping 编号用于排序（ping_list 是元胞数组）
                ping_numbers = zeros(1, length(ping_list));
                for i = 1:length(ping_list)
                    ping_numbers(i) = ping_list{i}.number;
                end
                [~, idx] = sort(ping_numbers);
                
                % 重新排列 ping_list（元胞数组）
                ping_list = ping_list(idx);
                
                if verbose
                    fprintf('找到 %d 个 ping 数据\n', length(ping_list));
                end
                
                % 确定要读取的 ping 索引
                if isempty(ping_indices)
                    ping_indices = 1:length(ping_list);
                else
                    % 验证索引有效性
                    ping_indices = ping_indices(ping_indices >= 1 & ping_indices <= length(ping_list));
                end
                
                % 读取每个 ping
                for list_idx = ping_indices
                    if list_idx <= length(ping_list)
                        ping_item = ping_list{list_idx};  % ping_list 是元胞数组
                        ping_path = [group_name '/' ping_item.name];
                        
                        if verbose
                            fprintf('读取 ping_%d (%s) 从路径 %s...\n', ping_item.number, ping_item.type, ping_path);
                        end
                        
                        ping_data = ReadDataset(filename, ping_path, verbose);
                        if ~isempty(ping_data)
                            data.pings{end+1} = ping_data;
                            if verbose
                                fprintf('  成功读取，大小: %s\n', mat2str(size(ping_data)));
                            end
                        else
                            if verbose
                                warning('ping_%d 数据为空', ping_item.number);
                            end
                        end
                    end
                end
            else
                if verbose
                    warning('未找到任何 ping 数据');
                end
            end
            
            if verbose
                fprintf('成功读取 %d 个 ping 数据\n', length(data.pings));
            end
            
        catch ME
            if verbose
                warning('无法读取 ping 数据: %s', E.message);
                fprintf('错误堆栈:\n');
                for k = 1:length(ME.stack)
                    fprintf('  %s (line %d)\n', ME.stack(k).name, ME.stack(k).line);
                end
            end
        end
    end
end

function attrs = ReadAttributes(filename, location, verbose)
% ReadAttributes - 读取指定位置的所有属性
    attrs = struct();
    
    try
        info = h5info(filename, location);
        
        % 读取组属性
        if isfield(info, 'Attributes')
            for i = 1:length(info.Attributes)
                attr_name = info.Attributes(i).Name;
                try
                    attr_value = h5readatt(filename, location, attr_name);
                    attrs.(attr_name) = attr_value;
                catch
                    if verbose
                        warning('无法读取属性 %s', attr_name);
                    end
                end
            end
        end
        
        % 读取数据集（如 receive_array_position, match_filter_data）
        if isfield(info, 'Datasets')
            for i = 1:length(info.Datasets)
                ds_name = info.Datasets(i).Name;
                ds_path = [location '/' ds_name];
                try
                    ds_data = ReadDataset(filename, ds_path, verbose);
                    attrs.(ds_name) = ds_data;
                catch
                    if verbose
                        warning('无法读取数据集 %s', ds_name);
                    end
                end
            end
        end
        
        % 检查子组中的数据集（如 match_filter_data/real, match_filter_data/imag）
        if isfield(info, 'Groups')
            for i = 1:length(info.Groups)
                grp_name = info.Groups(i).Name;
                grp_info = h5info(filename, grp_name);
                
                % 检查是否有 real 和 imag 数据集（复数数据）
                has_real = false;
                has_imag = false;
                real_path = '';
                imag_path = '';
                
                if isfield(grp_info, 'Datasets')
                    for j = 1:length(grp_info.Datasets)
                        if strcmp(grp_info.Datasets(j).Name, 'real')
                            has_real = true;
                            real_path = [grp_name '/real'];
                        elseif strcmp(grp_info.Datasets(j).Name, 'imag')
                            has_imag = true;
                            imag_path = [grp_name '/imag'];
                        end
                    end
                end
                
                if has_real && has_imag
                    % 复数数据：组合 real 和 imag
                    try
                        real_data = h5read(filename, real_path);
                        imag_data = h5read(filename, imag_path);
                        % 使用更兼容的方式提取名称
                        if length(grp_name) > length(location) + 1
                            ds_name = grp_name(length(location)+2:end);
                        else
                            ds_name = grp_name;
                        end
                        attrs.(ds_name) = real_data + 1i * imag_data;
                    catch ME
                        if verbose
                            warning('无法读取复数数据集 %s: %s', ds_name, ME.message);
                        end
                    end
                end
            end
        end
        
    catch ME
        if verbose
            warning('读取 attributes 时出错: %s', ME.message);
        end
    end
end

function data = ReadDataset(filename, dataset_path, verbose)
% ReadDataset - 读取数据集，自动处理复数数据
    try
        % 首先检查路径是组还是数据集
        try
            info = h5info(filename, dataset_path);
            is_group = isfield(info, 'Groups') || isfield(info, 'Datasets');
        catch
            is_group = false;
        end
        
        % 如果是组，检查是否有 real 和 imag 子数据集（复数数据）
        if is_group
            try
                info = h5info(filename, dataset_path);
                real_exists = false;
                imag_exists = false;
                real_path = '';
                imag_path = '';
                
                if isfield(info, 'Datasets')
                    for i = 1:length(info.Datasets)
                        if strcmp(info.Datasets(i).Name, 'real')
                            real_exists = true;
                            real_path = [dataset_path '/real'];
                        elseif strcmp(info.Datasets(i).Name, 'imag')
                            imag_exists = true;
                            imag_path = [dataset_path '/imag'];
                        end
                    end
                end
                
                if real_exists && imag_exists
                    % 复数数据：读取 real 和 imag
                    real_data = h5read(filename, real_path);
                    imag_data = h5read(filename, imag_path);
                    data = real_data + 1i * imag_data;
                    return;
                end
            catch ME
                if verbose
                    warning('检查复数数据时出错: %s', ME.message);
                end
            end
        end
        
        % 检查是否存在 complex 属性（可能在组上）
        try
            is_complex = h5readatt(filename, dataset_path, 'complex');
            if is_complex == 1
                % 标记为复数，尝试读取 real 和 imag
                try
                    real_path = [dataset_path '/real'];
                    imag_path = [dataset_path '/imag'];
                    real_data = h5read(filename, real_path);
                    imag_data = h5read(filename, imag_path);
                    data = real_data + 1i * imag_data;
                    return;
                catch
                    % 如果读取失败，继续尝试直接读取
                end
            end
        catch
            % 属性不存在，继续
        end
        
        % 普通数据：直接读取（可能是数据集）
        try
            data = h5read(filename, dataset_path);
        catch ME
            % 如果直接读取失败，可能是组但没有 real/imag，返回空
            if verbose
                warning('无法读取数据集 %s: %s', dataset_path, ME.message);
            end
            data = [];
        end
        
    catch ME
        if verbose
            warning('读取数据集 %s 时出错: %s', dataset_path, ME.message);
        end
        data = [];
    end
end

