classdef esl3d
    %ESL3D Read .esl3d file into memory and expose frame point clouds.
    %
    % Usage:
    %   d = esl3d("./Point Cloud/demo.esl3d");
    %   n = d.FrameCount;
    %   [pts, inten] = d.getPointCloud(1);
    %
    % pts   : N x 3 (x, y, z), unit meter
    % inten : N x 1 intensity

    properties (SetAccess = private)
        FilePath string
        FrameCount double = 0
    end

    properties (Access = private)
        Frames struct = struct( ...
            "seq", {}, ...
            "ts_us", {}, ...
            "range", {}, ...
            "intensity", {}, ...
            "metadata", {}, ...
            "sonar_cfg", {}, ...
            "env", {}, ...
            "pose", {})
    end

    methods
        function obj = esl3d(esl3dPath)
            if nargin < 1 || strlength(string(esl3dPath)) == 0
                error("Please provide a valid .esl3d file path.");
            end

            obj.FilePath = string(esl3dPath);
            obj.Frames = obj.readEsl3dPackets(char(obj.FilePath));
            obj.FrameCount = numel(obj.Frames);

            if obj.FrameCount == 0
                error("No frames found in file: %s", obj.FilePath);
            end
        end

        function n = getFrameCount(obj)
            n = obj.FrameCount;
        end

        function fr = getFrame(obj, idx)
            obj.validateFrameIndex(idx);
            fr = obj.Frames(idx);
        end

        function [points, intensity] = getPointCloud(obj, idx)
            obj.validateFrameIndex(idx);
            fr = obj.Frames(idx);
            [points, intensity] = obj.rangeToPointCloud(fr.range, fr.intensity, fr.sonar_cfg);
        end

        function [rangeImg, intensityImg] = getImages(obj, idx)
            obj.validateFrameIndex(idx);
            fr = obj.Frames(idx);
            rangeImg = fr.range;
            intensityImg = fr.intensity;
        end
    end

    methods (Access = private)
        function validateFrameIndex(obj, idx)
            if ~isscalar(idx) || ~isnumeric(idx) || ~isfinite(idx)
                error("Frame index must be a finite numeric scalar.");
            end
            idx = round(double(idx));
            if idx < 1 || idx > obj.FrameCount
                error("Frame index out of range. Expected 1..%d, got %d.", obj.FrameCount, idx);
            end
        end

        function frames = readEsl3dPackets(obj, esl3dPath)
            MAGIC = uint32(hex2dec("5033534E"));
            HEADER_SIZE = 56; % <IHHQQIIIIIIII

            fid = fopen(esl3dPath, "rb");
            if fid < 0
                error("Cannot open file: %s", esl3dPath);
            end
            cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>

            frames = struct( ...
                "seq", {}, ...
                "ts_us", {}, ...
                "range", {}, ...
                "intensity", {}, ...
                "metadata", {}, ...
                "sonar_cfg", {}, ...
                "env", {}, ...
                "pose", {});

            frameIdx = 0;
            while true
                headerRaw = fread(fid, HEADER_SIZE, "*uint8");
                if isempty(headerRaw)
                    break;
                end
                if numel(headerRaw) ~= HEADER_SIZE
                    error("Incomplete packet header at frame %d.", frameIdx + 1);
                end

                h = obj.parseHeader(headerRaw);
                obj.validateHeader(h, MAGIC, HEADER_SIZE);

                payload = fread(fid, double(h.payload_bytes), "*uint8");
                if numel(payload) ~= h.payload_bytes
                    error("Unexpected EOF while reading payload at frame %d.", frameIdx + 1);
                end

                metaRaw = payload(1:h.metadata_bytes);
                rangeStart = h.metadata_bytes + 1;
                rangeEnd = h.metadata_bytes + h.range_bytes;
                intensityStart = rangeEnd + 1;
                intensityEnd = rangeEnd + h.intensity_bytes;

                rangeRaw = payload(rangeStart:rangeEnd);
                intensityRaw = payload(intensityStart:intensityEnd);

                metadata = jsondecode(native2unicode(metaRaw', "UTF-8"));
                rangeImg = typecast(rangeRaw, "single");
                intensityImg = typecast(intensityRaw, "single");
                rangeImg = reshape(rangeImg, [double(h.width), double(h.height)])';
                intensityImg = reshape(intensityImg, [double(h.width), double(h.height)])';

                frameIdx = frameIdx + 1;
                frames(frameIdx).seq = h.seq;
                frames(frameIdx).ts_us = h.ts_us;
                frames(frameIdx).range = rangeImg;
                frames(frameIdx).intensity = intensityImg;
                frames(frameIdx).metadata = metadata;
                frames(frameIdx).sonar_cfg = obj.getStructField(metadata, "sonar_config", struct());
                frames(frameIdx).env = obj.getStructField(metadata, "environment", struct());
                frames(frameIdx).pose = obj.getStructField(metadata, "pose", struct());
            end
        end

        function h = parseHeader(obj, headerRaw) %#ok<INUSD>
            p = 1;
            h.magic = obj.readU32(headerRaw, p); p = p + 4;
            h.version = obj.readU16(headerRaw, p); p = p + 2;
            h.header_bytes = obj.readU16(headerRaw, p); p = p + 2;
            h.seq = obj.readU64(headerRaw, p); p = p + 8;
            h.ts_us = obj.readU64(headerRaw, p); p = p + 8;
            h.width = obj.readU32(headerRaw, p); p = p + 4;
            h.height = obj.readU32(headerRaw, p); p = p + 4;
            h.point_count = obj.readU32(headerRaw, p); p = p + 4;
            h.metadata_bytes = obj.readU32(headerRaw, p); p = p + 4;
            h.range_bytes = obj.readU32(headerRaw, p); p = p + 4;
            h.intensity_bytes = obj.readU32(headerRaw, p); p = p + 4;
            h.payload_bytes = obj.readU32(headerRaw, p); p = p + 4;
            h.reserved = obj.readU32(headerRaw, p);
        end

        function validateHeader(obj, h, MAGIC, HEADER_SIZE) %#ok<INUSD>
            if h.magic ~= MAGIC
                error("Bad magic: 0x%08X", h.magic);
            end
            if h.version ~= 1
                error("Unsupported version: %d", h.version);
            end
            if h.header_bytes ~= HEADER_SIZE
                error("Unexpected header size: %d", h.header_bytes);
            end
            if h.payload_bytes ~= (h.metadata_bytes + h.range_bytes + h.intensity_bytes)
                error("Payload length mismatch.");
            end
        end

        function [points, colors] = rangeToPointCloud(obj, rangeImg, intensityImg, sonarCfg) %#ok<INUSD>
            hfov = deg2rad(obj.getNumericField(sonarCfg, "horizontal_fov_deg", 90.0));
            vfov = deg2rad(obj.getNumericField(sonarCfg, "vertical_fov_deg", 30.0));

            [h, w] = size(rangeImg);
            az = linspace(-hfov * 0.5, hfov * 0.5, w);
            el = linspace(-vfov * 0.5, vfov * 0.5, h);
            [AZ, EL] = meshgrid(az, el);

            valid = isfinite(rangeImg) & (rangeImg > 0);
            if ~any(valid, "all")
                points = zeros(0, 3, "single");
                colors = zeros(0, 1, "single");
                return;
            end

            r = single(rangeImg(valid));
            azv = single(AZ(valid));
            elv = single(EL(valid));

            z = r .* cos(elv) .* cos(azv);
            x = r .* cos(elv) .* sin(azv);
            y = r .* sin(elv);

            points = [x, y, z];
            colors = single(intensityImg(valid));
        end
    end

    methods (Static, Access = private)
        function val = getStructField(s, fieldName, defaultVal)
            if isstruct(s) && isfield(s, fieldName)
                val = s.(fieldName);
            else
                val = defaultVal;
            end
        end

        function v = getNumericField(s, name, defaultV)
            if isstruct(s) && isfield(s, name) && isnumeric(s.(name)) && isscalar(s.(name))
                v = double(s.(name));
            else
                v = defaultV;
            end
        end

        function v = readU16(buf, p)
            v = typecast(buf(p:p+1), "uint16");
        end

        function v = readU32(buf, p)
            v = typecast(buf(p:p+3), "uint32");
        end

        function v = readU64(buf, p)
            v = typecast(buf(p:p+7), "uint64");
        end
    end
end
