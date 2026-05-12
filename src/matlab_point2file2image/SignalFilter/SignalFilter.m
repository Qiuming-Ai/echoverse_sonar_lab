classdef SignalFilter < handle
    %SIGNALFILTER FIR-based designer and processor for common responses.
    %
    %   filterObj = SignalFilter(Name,Value,...) builds a reusable FIR filter
    %   instance. Key parameters:
    %       'SampleRate'        : Sampling frequency in Hz (required)
    %       'Passband'          : Passband edge(s) in Hz. Scalar for low/high-pass,
    %                             two-element vector [F1 F2] for band-pass (required)
    %       'StopbandAtten'     : Minimum stopband attenuation in dB (required)
    %       'Type'              : 'lowpass' | 'highpass' | 'bandpass' (required)
    %       'SidebandMode'      : 'double' (default) | 'single'. Use 'single' if the
    %                             supplied spectral specs are one-sided (e.g.
    %                             analytic signals). Internally the class scales the
    %                             frequencies to match a two-sided design.
    %       'OutputSameLength'  : logical flag controlling whether PROCESS returns
    %                             the same-length result (default true). If false,
    %                             the full convolution is returned.
    %       'TransitionWidth'   : Optional transition width(s) in Hz. Scalar applied
    %                             symmetrically; two-element vector for band-pass
    %                             [lowerWidth upperWidth]. Defaults to 10%% of the
    %                             available transition band.
    %       'PassbandRipple'    : Optional passband ripple (dB). Default 0.5 dB.
    %       'DesignMethod'      : designfilt method, default 'kaiserwin'.
    %
    %   y = filterObj.process(x) filters the input signal x according to the
    %   configured FIR design. If OutputSameLength is true, the output size matches
    %   the input; otherwise the full convolution is returned.
    %
    %   info = filterObj.getInfo() returns a struct summarising the design, including
    %   coefficients, order, and derived specifications.
    %
    %   filterObj.plotResponse() visualises the magnitude response via FVTool.
    %
    %   This class relies on the Signal Processing Toolbox (designfilt/fvtool).

    properties (SetAccess = private)
        SampleRate           (1,1) double {mustBePositive} = 1
        Passband             (1,:) double = []
        StopbandAtten        (1,1) double {mustBePositive} = 1
        Type                 (1,:) char = ''
        SidebandMode         (1,:) char = 'double'
        OutputSameLength     (1,1) logical = true
        FilterOrientation    (1,:) char = 'columns'
        TransitionWidth      (1,:) double = []
        PassbandRipple       (1,1) double {mustBePositive} = 0.5
        DesignMethod         (1,:) char = 'kaiserwin'
    end

    properties (Access = private)
        FilterObject
        FilterCoeffs double = []
        EffectiveSampleRate double {mustBePositive}
        EffectivePassband (1,:) double {mustBeNonnegative}
        EffectiveTransitionWidth (1,:) double {mustBeNonnegative} = []
        ImpulseResponse double = []
        Order double = []
    end

    methods
        function obj = SignalFilter(varargin)
            parser = inputParser;
            parser.FunctionName = 'SignalFilter constructor';
            addParameter(parser, 'SampleRate', [], @(x) validateattributes(x, {'numeric'}, {'scalar','positive','real'}));
            addParameter(parser, 'Passband', [], @(x) validateattributes(x, {'numeric'}, {'vector','real','nonnegative'}));
            addParameter(parser, 'StopbandAtten', [], @(x) validateattributes(x, {'numeric'}, {'scalar','positive','real'}));
            addParameter(parser, 'Type', '', @(x) any(strcmpi(x, {'lowpass','highpass','bandpass'})));
            addParameter(parser, 'SidebandMode', 'double', @(x) any(strcmpi(x, {'single','double'})));
            addParameter(parser, 'OutputSameLength', true, @(x) islogical(x) && isscalar(x));
            addParameter(parser, 'TransitionWidth', [], @(x) validateattributes(x, {'numeric'}, {'vector','real','nonnegative'}));
            addParameter(parser, 'PassbandRipple', 0.5, @(x) validateattributes(x, {'numeric'}, {'scalar','positive','real'}));
            addParameter(parser, 'DesignMethod', 'kaiserwin', @(x) ischar(x) || isStringScalar(x));
            addParameter(parser, 'FilterOrientation', 'columns', @(x) any(strcmpi(x, {'columns','rows'})));
            parse(parser, varargin{:});

            obj.SampleRate = parser.Results.SampleRate;
            obj.Passband = parser.Results.Passband(:).';
            obj.StopbandAtten = parser.Results.StopbandAtten;
            obj.Type = lower(char(parser.Results.Type));
            obj.SidebandMode = lower(char(parser.Results.SidebandMode));
            obj.OutputSameLength = parser.Results.OutputSameLength;
            obj.FilterOrientation = lower(char(parser.Results.FilterOrientation));
            obj.TransitionWidth = parser.Results.TransitionWidth(:).';
            obj.PassbandRipple = parser.Results.PassbandRipple;
            obj.DesignMethod = char(parser.Results.DesignMethod);

            obj.validateCoreInputs();
            obj.computeEffectiveSpecs();
            obj.designInternalFilter();
        end

        function y = process(obj, x)
            arguments
                obj
                x {mustBeNumeric}
            end

            if isempty(obj.FilterCoeffs)
                error('SignalFilter:FilterNotDesigned', 'Filter coefficients are empty. Re-run designInternalFilter().');
            end

            [data, dim] = obj.prepareInputData(x);

            if obj.OutputSameLength
                y = filter(obj.FilterCoeffs, 1, data, [], dim);
            else
                y = obj.fullConvolution(data, dim);
            end

            y = obj.restoreOrientation(y, x, dim);
        end

        function info = getInfo(obj)
            info = struct( ...
                'SampleRate', obj.SampleRate, ...
                'EffectiveSampleRate', obj.EffectiveSampleRate, ...
                'Type', obj.Type, ...
                'SidebandMode', obj.SidebandMode, ...
                'SpecificationPassband', obj.Passband, ...
                'SpecificationTransitionWidth', obj.TransitionWidth, ...
                'EffectivePassband', obj.EffectivePassband, ...
                'EffectiveTransitionWidth', obj.EffectiveTransitionWidth, ...
                'FilterOrientation', obj.FilterOrientation, ...
                'StopbandAtten', obj.StopbandAtten, ...
                'PassbandRipple', obj.PassbandRipple, ...
                'DesignMethod', obj.DesignMethod, ...
                'Order', obj.Order, ...
                'Coefficients', obj.FilterCoeffs, ...
                'ImpulseResponse', obj.ImpulseResponse, ...
                'NumTaps', numel(obj.FilterCoeffs));
        end

        function response = getImpulseResponse(obj, varargin)
            parser = inputParser;
            parser.FunctionName = 'SignalFilter.getImpulseResponse';
            addParameter(parser, 'Length', [], @(x) isempty(x) || (isscalar(x) && x > 0 && fix(x) == x));
            parse(parser, varargin{:});

            n = parser.Results.Length;
            if isempty(n)
                h = obj.ImpulseResponse;
            else
                impulse = [1, zeros(1, n - 1)];
                h = filter(obj.FilterCoeffs, 1, impulse);
            end

            dt = 1 / obj.EffectiveSampleRate;
            t = (0:numel(h)-1) .* dt;

            response = struct('Time', t, 'Amplitude', h);
        end

        function plotResponse(obj)
            if isempty(obj.FilterCoeffs)
                error('SignalFilter:FilterNotDesigned', 'Filter coefficients are empty. Re-run designInternalFilter().');
            end
            fvtool(obj.FilterCoeffs, 1, 'Fs', obj.EffectiveSampleRate);
        end

        function redesign(obj, varargin)
            if ~isempty(varargin)
                parser = inputParser;
                parser.FunctionName = 'SignalFilter.redesign';
                addParameter(parser, 'Passband', obj.Passband, @(x) validateattributes(x, {'numeric'}, {'vector','real','nonnegative'}));
                addParameter(parser, 'TransitionWidth', obj.TransitionWidth, @(x) validateattributes(x, {'numeric'}, {'vector','real','nonnegative'}));
                addParameter(parser, 'StopbandAtten', obj.StopbandAtten, @(x) validateattributes(x, {'numeric'}, {'scalar','positive','real'}));
                addParameter(parser, 'PassbandRipple', obj.PassbandRipple, @(x) validateattributes(x, {'numeric'}, {'scalar','positive','real'}));
                parse(parser, varargin{:});

                obj.Passband = parser.Results.Passband(:).';
                obj.TransitionWidth = parser.Results.TransitionWidth(:).';
                obj.StopbandAtten = parser.Results.StopbandAtten;
                obj.PassbandRipple = parser.Results.PassbandRipple;
            end
            obj.computeEffectiveSpecs();
            obj.designInternalFilter();
        end
    end

    methods (Access = private)
        function validateCoreInputs(obj)
            if isempty(obj.SampleRate)
                error('SignalFilter:MissingSampleRate', 'SampleRate must be specified.');
            end
            if isempty(obj.Passband)
                error('SignalFilter:MissingPassband', 'Passband must be specified.');
            end

            switch obj.Type
                case {'lowpass', 'highpass'}
                    if numel(obj.Passband) ~= 1
                        error('SignalFilter:InvalidPassband', 'Passband must be scalar for %s filters.', obj.Type);
                    end
                case 'bandpass'
                    if numel(obj.Passband) ~= 2
                        error('SignalFilter:InvalidPassband', 'Passband must be a two-element vector for bandpass filters.');
                    end
                    if obj.Passband(1) >= obj.Passband(2)
                        error('SignalFilter:InvalidPassbandOrder', 'Bandpass Passband(1) must be less than Passband(2).');
                    end
            end
        end

        function computeEffectiveSpecs(obj)
            scale = 1;
            if strcmp(obj.SidebandMode, 'single')
                scale = 0.5;
            end

            obj.EffectiveSampleRate = obj.SampleRate;
            obj.EffectivePassband = obj.Passband * scale;
            if isempty(obj.TransitionWidth)
                obj.EffectiveTransitionWidth = [];
            else
                obj.EffectiveTransitionWidth = obj.TransitionWidth * scale;
            end
        end

        function designInternalFilter(obj)
            nyquist = obj.EffectiveSampleRate / 2;

            switch obj.Type
                case 'lowpass'
                    [passband, stopband] = localLowpassBands(obj, nyquist);
                    obj.FilterObject = designfilt( ...
                        'lowpassfir', ...
                        'PassbandFrequency', passband, ...
                        'StopbandFrequency', stopband, ...
                        'PassbandRipple', obj.PassbandRipple, ...
                        'StopbandAttenuation', obj.StopbandAtten, ...
                        'DesignMethod', obj.DesignMethod, ...
                        'SampleRate', obj.EffectiveSampleRate);

                case 'highpass'
                    [passband, stopband] = localHighpassBands(obj, nyquist);
                    obj.FilterObject = designfilt( ...
                        'highpassfir', ...
                        'PassbandFrequency', passband, ...
                        'StopbandFrequency', stopband, ...
                        'PassbandRipple', obj.PassbandRipple, ...
                        'StopbandAttenuation', obj.StopbandAtten, ...
                        'DesignMethod', obj.DesignMethod, ...
                        'SampleRate', obj.EffectiveSampleRate);

                case 'bandpass'
                    [passband, stopband] = localBandpassBands(obj, nyquist);
                    obj.FilterObject = designfilt( ...
                        'bandpassfir', ...
                        'PassbandFrequency1', passband(1), ...
                        'PassbandFrequency2', passband(2), ...
                        'StopbandFrequency1', stopband(1), ...
                        'StopbandFrequency2', stopband(2), ...
                        'PassbandRipple', obj.PassbandRipple, ...
                        'StopbandAttenuation1', obj.StopbandAtten, ...
                        'StopbandAttenuation2', obj.StopbandAtten, ...
                        'DesignMethod', obj.DesignMethod, ...
                        'SampleRate', obj.EffectiveSampleRate);
            end

            obj.FilterCoeffs = obj.FilterObject.Coefficients(:).';
            obj.Order = numel(obj.FilterCoeffs) - 1;
            obj.ImpulseResponse = obj.FilterCoeffs;
        end

        function [data, dim] = prepareInputData(obj, x)
            if isvector(x)
                switch obj.FilterOrientation
                    case 'columns'
                        data = x(:);
                        dim = 1;
                    case 'rows'
                        data = x(:).';
                        dim = 2;
                end
            else
                data = x;
                dim = obj.selectDimension();
            end
        end

        function dim = selectDimension(obj)
            if strcmp(obj.FilterOrientation, 'columns')
                dim = 1;
            else
                dim = 2;
            end
        end

        function y = fullConvolution(obj, data, dim)
            if dim == 1
                nCols = size(data, 2);
                y = zeros(size(data,1) + numel(obj.FilterCoeffs) - 1, nCols);
                for c = 1:nCols
                    y(:, c) = conv(data(:, c), obj.FilterCoeffs, 'full');
                end
            else
                nRows = size(data, 1);
                y = zeros(nRows, size(data,2) + numel(obj.FilterCoeffs) - 1);
                for r = 1:nRows
                    y(r, :) = conv(data(r, :), obj.FilterCoeffs, 'full');
                end
            end
        end

        function y = restoreOrientation(obj, y, originalInput, dim)
            if isvector(originalInput)
                if strcmp(obj.FilterOrientation, 'columns')
                    y = y(:);
                else
                    y = y(:).';
                end
            else
                if dim == 1 && strcmp(obj.FilterOrientation, 'rows')
                    y = y.';
                elseif dim == 2 && strcmp(obj.FilterOrientation, 'columns')
                    y = y.';
                end
            end
        end

        function [passband, stopband] = localLowpassBands(obj, nyquist)
            passband = obj.EffectivePassband;
            width = obj.resolveTransitionWidth(1, nyquist - passband);
            stopband = min(passband + width, nyquist * 0.999);
        end

        function [passband, stopband] = localHighpassBands(obj, nyquist)
            passband = obj.EffectivePassband;
            width = obj.resolveTransitionWidth(1, passband);
            stopband = max(passband - width, nyquist * 0.001);
        end

        function [passband, stopband] = localBandpassBands(obj, nyquist)
            passband = obj.EffectivePassband;
            width = obj.resolveTransitionWidth(2, [passband(1), nyquist - passband(2)]);
            stopband = [ ...
                max(passband(1) - width(1), nyquist * 0.001), ...
                min(passband(2) + width(2), nyquist * 0.999)];
        end

        function width = resolveTransitionWidth(obj, count, limits)
            if isempty(obj.EffectiveTransitionWidth)
                defaultWidth = 0.1 .* limits;
                defaultWidth = max(defaultWidth, limits * 0.05 + eps);
                width = defaultWidth;
            else
                if numel(obj.EffectiveTransitionWidth) == 1
                    width = repmat(obj.EffectiveTransitionWidth, 1, count);
                elseif numel(obj.EffectiveTransitionWidth) == count
                    width = obj.EffectiveTransitionWidth;
                else
                    error('SignalFilter:TransitionWidthSize', ...
                        'TransitionWidth must be scalar or match the number of edges.');
                end
            end
            width = min(width, limits - eps);
            width = max(width, eps);
        end
    end
end

