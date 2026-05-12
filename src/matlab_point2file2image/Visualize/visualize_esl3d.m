function visualize_esl3d(esl3dPath)
%VISUALIZE_ESL3D Read .esl3d frames and visualize range/intensity/point cloud.
% Usage:
%   visualize_esl3d
%   visualize_esl3d("./Point Cloud/demo.esl3d")

    if nargin < 1 || strlength(string(esl3dPath)) == 0
        [fname, fpath] = uigetfile("*.esl3d", "Select an .esl3d file");
        if isequal(fname, 0)
            fprintf("No .esl3d file selected.\n");
            return;
        end
        esl3dPath = fullfile(fpath, fname);
    end

    frames = readEsl3dPackets(esl3dPath);
    if isempty(frames)
        error("No frames found in file: %s", esl3dPath);
    end

    fprintf("Loaded %d frame(s) from %s\n", numel(frames), esl3dPath);
    showFrameMetadata(frames(1), 1);

    buildViewer(frames, esl3dPath);
end

function frames = readEsl3dPackets(esl3dPath)
    MAGIC = uint32(hex2dec("5033534E"));
    HEADER_SIZE = 56; % <IHHQQIIIIIIII

    fid = fopen(esl3dPath, "rb");
    if fid < 0
        error("Cannot open file: %s", esl3dPath);
    end
    cleaner = onCleanup(@() fclose(fid));

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

        h = parseHeader(headerRaw);
        validateHeader(h, MAGIC, HEADER_SIZE);

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
        frames(frameIdx).sonar_cfg = getStructField(metadata, "sonar_config", struct());
        frames(frameIdx).env = getStructField(metadata, "environment", struct());
        frames(frameIdx).pose = getStructField(metadata, "pose", struct());
    end
end

function h = parseHeader(headerRaw)
    p = 1;
    h.magic = readU32(headerRaw, p); p = p + 4;
    h.version = readU16(headerRaw, p); p = p + 2;
    h.header_bytes = readU16(headerRaw, p); p = p + 2;
    h.seq = readU64(headerRaw, p); p = p + 8;
    h.ts_us = readU64(headerRaw, p); p = p + 8;
    h.width = readU32(headerRaw, p); p = p + 4;
    h.height = readU32(headerRaw, p); p = p + 4;
    h.point_count = readU32(headerRaw, p); p = p + 4;
    h.metadata_bytes = readU32(headerRaw, p); p = p + 4;
    h.range_bytes = readU32(headerRaw, p); p = p + 4;
    h.intensity_bytes = readU32(headerRaw, p); p = p + 4;
    h.payload_bytes = readU32(headerRaw, p); p = p + 4;
    h.reserved = readU32(headerRaw, p);
end

function validateHeader(h, MAGIC, HEADER_SIZE)
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

function buildViewer(frames, esl3dPath)
    nFrames = numel(frames);
    fig = figure("Name", "ESL3D Viewer", "Color", "w");
    fig.Position(3:4) = [1450 640];

    axRange = subplot(2, 3, 1);
    axIntensity = subplot(2, 3, 2);
    axCloud = subplot(2, 3, 3);
    axInfo = subplot(2, 3, [4 5 6]);
    axis(axInfo, "off");

    slider = uicontrol( ...
        "Style", "slider", ...
        "Units", "normalized", ...
        "Position", [0.18 0.02 0.64 0.035], ...
        "Min", 1, ...
        "Max", max(1, nFrames), ...
        "Value", 1, ...
        "SliderStep", [1/max(1, nFrames - 1), min(1, 10/max(1, nFrames - 1))], ...
        "Callback", @onSliderChanged);

    label = uicontrol( ...
        "Style", "text", ...
        "Units", "normalized", ...
        "Position", [0.02 0.02 0.14 0.035], ...
        "String", "Frame: 1", ...
        "HorizontalAlignment", "left", ...
        "BackgroundColor", "w");

    renderFrame(1);

    function onSliderChanged(src, ~)
        idx = max(1, min(nFrames, round(src.Value)));
        src.Value = idx;
        label.String = sprintf("Frame: %d/%d", idx, nFrames);
        renderFrame(idx);
    end

    function renderFrame(idx)
        fr = frames(idx);

        imagesc(axRange, fr.range);
        axis(axRange, "image");
        colormap(axRange, "parula");
        colorbar(axRange);
        title(axRange, sprintf("Range (frame %d)", idx));
        xlabel(axRange, "Column");
        ylabel(axRange, "Row");

        imagesc(axIntensity, fr.intensity);
        axis(axIntensity, "image");
        colormap(axIntensity, "hot");
        colorbar(axIntensity);
        title(axIntensity, sprintf("Intensity (frame %d)", idx));
        xlabel(axIntensity, "Column");
        ylabel(axIntensity, "Row");

        [pts, c] = rangeToPointCloud(fr.range, fr.intensity, fr.sonar_cfg);
        cla(axCloud);
        if isempty(pts)
            title(axCloud, "Point Cloud (no valid points)");
            grid(axCloud, "on");
            xlabel(axCloud, "X (m)"); ylabel(axCloud, "Y (m)"); zlabel(axCloud, "Z (m)");
        else
            scatter3(axCloud, pts(:,1), pts(:,2), pts(:,3), 4, c, "filled");
            colormap(axCloud, "hot");
            colorbar(axCloud);
            grid(axCloud, "on");
            axis(axCloud, "equal");
            xlabel(axCloud, "X (m)");
            ylabel(axCloud, "Y (m)");
            zlabel(axCloud, "Z (m)");
            title(axCloud, sprintf("Point Cloud (N=%d)", size(pts,1)));
            view(axCloud, 3);
        end

        infoText = buildInfoText(fr, idx, nFrames, esl3dPath);
        cla(axInfo);
        axis(axInfo, "off");
        text(axInfo, 0.01, 0.98, infoText, ...
            "Interpreter", "none", ...
            "VerticalAlignment", "top", ...
            "FontName", "Consolas", ...
            "FontSize", 10);
    end
end

function [points, colors] = rangeToPointCloud(rangeImg, intensityImg, sonarCfg)
    hfov = deg2rad(getNumericField(sonarCfg, "horizontal_fov_deg", 90.0));
    vfov = deg2rad(getNumericField(sonarCfg, "vertical_fov_deg", 30.0));

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

function txt = buildInfoText(fr, idx, nFrames, esl3dPath)
    pc = size(fr.range, 1) * size(fr.range, 2);
    validPc = nnz(isfinite(fr.range) & fr.range > 0);
    lines = [ ...
        "File: " + string(esl3dPath)
        sprintf("Frame: %d / %d", idx, nFrames)
        sprintf("Seq: %d", fr.seq)
        sprintf("Timestamp(us): %d", fr.ts_us)
        sprintf("Image Size: %d x %d", size(fr.range,2), size(fr.range,1))
        sprintf("Pixel Count: %d", pc)
        sprintf("Valid Range Points: %d", validPc)
        ""
        "Sonar Config:"
        string(structToPretty(fr.sonar_cfg))
        ""
        "Environment:"
        string(structToPretty(fr.env))
        ""
        "Pose:"
        string(structToPretty(fr.pose))
    ];
    txt = char(join(lines, newline));
end

function out = structToPretty(s)
    if isempty(s)
        out = "(empty)";
        return;
    end
    try
        out = jsonencode(s, "PrettyPrint", true);
    catch
        out = evalc("disp(s)");
    end
end

function showFrameMetadata(fr, idx)
    fprintf("\n--- Frame %d metadata ---\n", idx);
    fprintf("seq=%d, ts_us=%d, size=%dx%d\n", ...
        fr.seq, fr.ts_us, size(fr.range,2), size(fr.range,1));
    fprintf("sonar_cfg:\n%s\n", structToPretty(fr.sonar_cfg));
    fprintf("env:\n%s\n", structToPretty(fr.env));
    fprintf("pose:\n%s\n\n", structToPretty(fr.pose));
end

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
