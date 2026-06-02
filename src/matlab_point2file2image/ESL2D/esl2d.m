classdef esl2d
    %ESL2D Read .esl2d 2D sonar files (FLS / SSS beam-organized intensity).
    %
    % Usage:
    %   d = esl2d("./Sonar 2D/demo_fls_20260101_1200.esl2d");
    %   n = d.FrameCount;
    %   fr = d.getFrame(1);
    %   img = d.getBeamIntensityMatrix(1);   % [beam_count x bin_count]
    %
    % FLS example: 256 beams, bearings ~[-65, +65] deg, bins along 0..max_range_m.
    % SSS example: 2 beams (starboard 0 deg, port 180 deg).

    properties (SetAccess = private)
        FilePath string
        FrameCount double = 0
    end

    properties (Access = private)
        Frames struct = struct( ...
            "seq", {}, ...
            "ts_us", {}, ...
            "sonar_type", {}, ...
            "sonar_kind", {}, ...
            "beam_count", {}, ...
            "bin_count", {}, ...
            "max_range_m", {}, ...
            "beam_angles_deg", {}, ...
            "intensity", {}, ...
            "metadata", {}, ...
            "sonar_cfg", {}, ...
            "env", {}, ...
            "pose", {}, ...
            "beams", {})
    end

    methods
        function obj = esl2d(esl2dPath)
            if nargin < 1 || strlength(string(esl2dPath)) == 0
                error("Please provide a valid .esl2d file path.");
            end

            obj.FilePath = string(esl2dPath);
            obj.Frames = obj.readEsl2dPackets(char(obj.FilePath));
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

        function mat = getBeamIntensityMatrix(obj, idx)
            obj.validateFrameIndex(idx);
            fr = obj.Frames(idx);
            mat = fr.intensity;
        end

        function ranges = getBinRangeAxis(obj, idx)
            obj.validateFrameIndex(idx);
            fr = obj.Frames(idx);
            n = double(fr.bin_count);
            ranges = ((0:n-1) + 0.5) / n * double(fr.max_range_m);
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

        function frames = readEsl2dPackets(obj, esl2dPath)
            MAGIC = uint32(hex2dec("5032534E"));
            HEADER_SIZE = 64;

            fid = fopen(esl2dPath, "rb");
            if fid < 0
                error("Cannot open file: %s", esl2dPath);
            end
            cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>

            frames = struct( ...
                "seq", {}, ...
                "ts_us", {}, ...
                "sonar_type", {}, ...
                "sonar_kind", {}, ...
                "beam_count", {}, ...
                "bin_count", {}, ...
                "max_range_m", {}, ...
                "beam_angles_deg", {}, ...
                "intensity", {}, ...
                "metadata", {}, ...
                "sonar_cfg", {}, ...
                "env", {}, ...
                "pose", {}, ...
                "beams", {});

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

                metaEnd = h.metadata_bytes;
                anglesEnd = metaEnd + h.beam_angles_bytes;
                intensityEnd = anglesEnd + h.intensity_bytes;

                metaRaw = payload(1:metaEnd);
                anglesRaw = payload(metaEnd+1:anglesEnd);
                intensityRaw = payload(anglesEnd+1:intensityEnd);

                metadata = jsondecode(native2unicode(metaRaw', "UTF-8"));
                beamAngles = typecast(anglesRaw, "single");
                intensityFlat = typecast(intensityRaw, "single");
                intensityMat = reshape(intensityFlat, [double(h.bin_count), double(h.beam_count)])';

                frameIdx = frameIdx + 1;
                frames(frameIdx).seq = h.seq;
                frames(frameIdx).ts_us = h.ts_us;
                frames(frameIdx).sonar_type = h.sonar_type;
                frames(frameIdx).sonar_kind = obj.getStructField(metadata, "sonar_kind", "");
                frames(frameIdx).beam_count = h.beam_count;
                frames(frameIdx).bin_count = h.bin_count;
                frames(frameIdx).max_range_m = h.max_range_m;
                frames(frameIdx).beam_angles_deg = beamAngles(:)';
                frames(frameIdx).intensity = intensityMat;
                frames(frameIdx).metadata = metadata;
                frames(frameIdx).sonar_cfg = obj.getStructField(metadata, "sonar_config", struct());
                frames(frameIdx).env = obj.getStructField(metadata, "environment", struct());
                frames(frameIdx).pose = obj.getStructField(metadata, "pose", struct());
                frames(frameIdx).beams = obj.getStructField(metadata, "beams", []);
            end
        end

        function h = parseHeader(obj, headerRaw) %#ok<INUSD>
            p = 1;
            h.magic = obj.readU32(headerRaw, p); p = p + 4;
            h.version = obj.readU16(headerRaw, p); p = p + 2;
            h.header_bytes = obj.readU16(headerRaw, p); p = p + 2;
            h.seq = obj.readU64(headerRaw, p); p = p + 8;
            h.ts_us = obj.readU64(headerRaw, p); p = p + 8;
            h.sonar_type = obj.readU16(headerRaw, p); p = p + 2;
            h.reserved0 = obj.readU16(headerRaw, p); p = p + 2;
            h.beam_count = obj.readU32(headerRaw, p); p = p + 4;
            h.bin_count = obj.readU32(headerRaw, p); p = p + 4;
            h.max_range_m = obj.readF32(headerRaw, p); p = p + 4;
            h.metadata_bytes = obj.readU32(headerRaw, p); p = p + 4;
            h.beam_angles_bytes = obj.readU32(headerRaw, p); p = p + 4;
            h.intensity_bytes = obj.readU32(headerRaw, p); p = p + 4;
            h.payload_bytes = obj.readU32(headerRaw, p); p = p + 4;
            h.reserved1 = obj.readU32(headerRaw, p); p = p + 4;
            h.reserved2 = obj.readU32(headerRaw, p);
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
            if h.payload_bytes ~= (h.metadata_bytes + h.beam_angles_bytes + h.intensity_bytes)
                error("Payload length mismatch.");
            end
            if h.beam_angles_bytes ~= 4 * h.beam_count
                error("beam_angles_bytes mismatch.");
            end
            if h.intensity_bytes ~= 4 * h.beam_count * h.bin_count
                error("intensity_bytes mismatch.");
            end
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

        function v = readU16(buf, p)
            v = typecast(buf(p:p+1), "uint16");
        end

        function v = readU32(buf, p)
            v = typecast(buf(p:p+3), "uint32");
        end

        function v = readU64(buf, p)
            v = typecast(buf(p:p+7), "uint64");
        end

        function v = readF32(buf, p)
            v = typecast(buf(p:p+3), "single");
        end
    end
end
