classdef SonarDataMaker < handle
    % SonarDataMaker Incrementally writes sonar waveform frames to HDF5.
    % Compatible with BaselineDataMaker/CDMDataMaker/FDMDataMaker output layout.

    properties (Access = private)
        filePath (1, :) char = ''
        rawGroup (1, :) char = '/raw_data'
        attrsGroup (1, :) char = '/raw_data/.attributes'
        started (1, 1) logical = false
        closed (1, 1) logical = false
        frameCount (1, 1) double = 0
    end

    methods
        function start(obj, filePath, sonarInfo)
            % start Initialize output HDF5 and write sonar attributes.
            %
            % Inputs:
            %   filePath  - output HDF5 file path
            %   sonarInfo - struct of sonar metadata/attributes

            if nargin < 3
                error('SonarDataMaker:start:InvalidInput', ...
                    'start requires filePath and sonarInfo.');
            end
            if ~ischar(filePath) && ~isstring(filePath)
                error('SonarDataMaker:start:InvalidPath', ...
                    'filePath must be char or string.');
            end
            if ~isstruct(sonarInfo)
                error('SonarDataMaker:start:InvalidSonarInfo', ...
                    'sonarInfo must be a struct.');
            end

            obj.assertNotClosed();
            if obj.started
                error('SonarDataMaker:start:AlreadyStarted', ...
                    'start has already been called.');
            end

            obj.filePath = char(filePath);
            if exist(obj.filePath, 'file')
                delete(obj.filePath);
            end

            obj.createFile();
            obj.ensureGroup(obj.rawGroup);
            obj.ensureGroup(obj.attrsGroup);

            obj.writeSonarInfo(sonarInfo);

            obj.started = true;
            obj.frameCount = 0;
        end

        function write(obj, waveform)
            % write Append one waveform frame to /raw_data/ping_i.
            obj.assertWritable();
            if isempty(waveform)
                error('SonarDataMaker:write:EmptyWaveform', ...
                    'waveform cannot be empty.');
            end
            if ~(isnumeric(waveform) || islogical(waveform))
                error('SonarDataMaker:write:InvalidWaveformType', ...
                    'waveform must be numeric or logical.');
            end

            obj.frameCount = obj.frameCount + 1;
            datasetName = sprintf('%s/ping_%d', obj.rawGroup, obj.frameCount);
            obj.writeNumericDatasetWithComplexSupport(datasetName, waveform);
        end

        function close(obj)
            % close Finalize writing and lock this writer.
            obj.assertNotClosed();
            if ~obj.started
                error('SonarDataMaker:close:NotStarted', ...
                    'start must be called before close.');
            end

            h5writeatt(obj.filePath, obj.attrsGroup, 'ping_num', obj.frameCount);
            obj.closed = true;
        end
    end

    methods (Access = private)
        function writeSonarInfo(obj, sonarInfo)
            fields = fieldnames(sonarInfo);
            for i = 1:numel(fields)
                key = fields{i};
                value = sonarInfo.(key);
                obj.writeField(key, value);
            end

            if ~isfield(sonarInfo, 'timestamp')
                h5writeatt(obj.filePath, obj.attrsGroup, 'timestamp', ...
                    datestr(now, 'yyyymmdd_HHMMSS'));
            end
        end

        function writeField(obj, key, value)
            if isstring(value) && isscalar(value)
                h5writeatt(obj.filePath, obj.attrsGroup, key, char(value));
                return;
            end
            if ischar(value)
                h5writeatt(obj.filePath, obj.attrsGroup, key, value);
                return;
            end

            if isnumeric(value) || islogical(value)
                if isscalar(value) && isreal(value)
                    % Keep scalar real metadata as attributes (same style as legacy makers).
                    h5writeatt(obj.filePath, obj.attrsGroup, key, double(value));
                else
                    datasetPath = sprintf('%s/%s', obj.attrsGroup, key);
                    obj.writeNumericDatasetWithComplexSupport(datasetPath, value);
                end
                return;
            end

            error('SonarDataMaker:start:UnsupportedFieldType', ...
                'Unsupported sonarInfo field type for key "%s".', key);
        end

        function writeNumericDatasetWithComplexSupport(obj, datasetPath, data)
            if isreal(data)
                dataToWrite = single(data);
                h5create(obj.filePath, datasetPath, size(dataToWrite), 'Datatype', 'single');
                h5write(obj.filePath, datasetPath, dataToWrite);
                h5writeatt(obj.filePath, datasetPath, 'complex', 0);
            else
                realPath = [datasetPath '/real'];
                imagPath = [datasetPath '/imag'];
                realData = single(real(data));
                imagData = single(imag(data));
                h5create(obj.filePath, realPath, size(realData), 'Datatype', 'single');
                h5write(obj.filePath, realPath, realData);
                h5create(obj.filePath, imagPath, size(imagData), 'Datatype', 'single');
                h5write(obj.filePath, imagPath, imagData);
                h5writeatt(obj.filePath, datasetPath, 'complex', 1);
            end
        end

        function createFile(obj)
            fid = H5F.create(obj.filePath, 'H5F_ACC_TRUNC', 'H5P_DEFAULT', 'H5P_DEFAULT');
            H5F.close(fid);
        end

        function ensureGroup(obj, groupPath)
            fid = H5F.open(obj.filePath, 'H5F_ACC_RDWR', 'H5P_DEFAULT');
            cleanupObj = onCleanup(@() H5F.close(fid));
            %#ok<NASGU>

            parts = split(groupPath, '/');
            current = '';
            for i = 1:numel(parts)
                token = parts{i};
                if strlength(token) == 0
                    continue;
                end
                current = [current '/' char(token)]; %#ok<AGROW>
                if ~obj.groupExists(fid, current)
                    gid = H5G.create(fid, current, 'H5P_DEFAULT', 'H5P_DEFAULT', 'H5P_DEFAULT');
                    H5G.close(gid);
                end
            end
        end

        function tf = groupExists(~, fid, groupPath)
            tf = false;
            try
                gid = H5G.open(fid, groupPath);
                H5G.close(gid);
                tf = true;
            catch
                tf = false;
            end
        end

        function assertWritable(obj)
            if ~obj.started
                error('SonarDataMaker:write:NotStarted', ...
                    'start must be called before write.');
            end
            obj.assertNotClosed();
        end

        function assertNotClosed(obj)
            if obj.closed
                error('SonarDataMaker:Closed', ...
                    'Writer is already closed.');
            end
        end
    end
end
