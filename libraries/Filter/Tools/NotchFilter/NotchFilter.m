
% notch implmentation
classdef NotchFilter
    properties (SetAccess = private)
        initialised = false;
        need_reset = false;
        b0 = single(0);
        b1 = single(0);
        b2 = single(0);
        a1 = single(0);
        a2 = single(0);
        a0_inv = single(0);
        center_freq_hz = single(0);
        sample_freq_hz = single(0);
        ntchsig = single(0);
        ntchsig1 = single(0);
        ntchsig2 = single(0);
        signal2 = single(0);
        signal1 = single(0);

        NOTCH_MAX_SLEW = single(0.05);
        NOTCH_MAX_SLEW_LOWER = 1.0 - single(0.05);
        NOTCH_MAX_SLEW_UPPER = 1.0 / (1 - single(0.05));
    end
    methods

        function [A, Q] = calculate_A_and_Q(~, center_freq_hz, bandwidth_hz, attenuation_dB)
            A = power(10, -attenuation_dB / 40.0);
            if center_freq_hz > (0.5 * bandwidth_hz)
                octaves = log2(center_freq_hz / (center_freq_hz - bandwidth_hz / 2.0)) * 2.0;
                Q = sqrt(power(2, octaves)) / (power(2, octaves) - 1.0);
            else
                Q = 0.0;
            end
        end

        function obj = init(obj, sample_freq_hz, center_freq_hz, bandwidth_hz, attenuation_dB)
            % check center frequency is in the allowable range
            if ((center_freq_hz > 0.5 * bandwidth_hz) && (center_freq_hz < 0.5 * sample_freq_hz))
                obj.initialised = false;    % force center frequency change
                [A, Q] = calculate_A_and_Q(obj, center_freq_hz, bandwidth_hz, attenuation_dB);
                obj = init_with_A_and_Q(obj, sample_freq_hz, center_freq_hz, A, Q);
            else
                obj.initialised = false;
            end
        end


        function obj = init_with_A_and_Q(obj, sample_freq_hz, center_freq_hz, A, Q)
            % don't update if no updates required
            if obj.initialised && (center_freq_hz == obj.center_freq_hz) && (sample_freq_hz ==  obj.sample_freq_hz)
                return;
            end

            new_center_freq = center_freq_hz;

            % constrain the new center frequency by a percentage of the old frequency
            if obj.initialised && ~obj.need_reset && (obj.center_freq_hz ~= 0)
                new_center_freq = max(new_center_freq, obj.center_freq_hz * obj.NOTCH_MAX_SLEW_LOWER);
                new_center_freq = min(new_center_freq, obj.center_freq_hz * obj.NOTCH_MAX_SLEW_UPPER);
            end

            if (new_center_freq > 0.0) && (new_center_freq < 0.5 * sample_freq_hz) && (Q > 0.0)
                omega = 2.0 * pi * new_center_freq / sample_freq_hz;
                alpha = sin(omega) / (2 * Q);
                obj.b0 =  1.0 + alpha*(A*A);
                obj.b1 = -2.0 * cos(omega);
                obj.b2 =  1.0 - alpha*(A*A);
                obj.a0_inv =  1.0/(1.0 + alpha);
                obj.a1 = obj.b1;
                obj.a2 = 1.0 - alpha;
                obj.center_freq_hz = new_center_freq;
                obj.sample_freq_hz = sample_freq_hz;
                obj.initialised = true;
            else
                % leave center_freq_hz at last value
                obj.initialised = false;
            end
        end


        function [obj, ret] = apply(obj, sample)
            if ~obj.initialised || obj.need_reset
                % if we have not been initialised when return the input
                % sample as output and update delayed samples
                obj.signal1 = sample;
                obj.signal2 = sample;
                obj.ntchsig = sample;
                obj.ntchsig1 = sample;
                obj.ntchsig2 = sample;
                obj.need_reset = false;
                ret = sample;
                return
            end
            obj.ntchsig2 = obj.ntchsig1;
            obj.ntchsig1 = obj.ntchsig;
            obj.ntchsig = sample;

            output = (obj.ntchsig*obj.b0 + obj.ntchsig1*obj.b1 + obj.ntchsig2*obj.b2 - obj.signal1*obj.a1 - obj.signal2*obj.a2) * obj.a0_inv;

            obj.signal2 = obj.signal1;
            obj.signal1 = output;
            ret = output;
        end


        function obj = reset(obj)
            obj.need_reset = true;
        end
    end
end













