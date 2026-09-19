% Octave/MATLAB script for the AccumulatorTransferFunction example.
%
% Run from this directory after generating test.csv:
%   octave AccumulatorTest.m
%
% The IMU runs at sample_rate and publishes every N samples at output_rate.
% The output is delta_angle (rad).  Transfer function from gyro rate (rad/s):
%
%   H(z) = (1/(2*sample_rate)) * (1+z^-1) * (1-z^-N) / (1-z^-1)
%
% where z is evaluated at the IMU sample rate.

close all
clear
clc

filename = 'test.csv';

% --- Read every line from the file ---
fid = fopen(filename, 'r');
if fid < 0
    error('Could not open %s', filename);
end
all_lines = {};
while true
    line = fgetl(fid);
    if ~ischar(line)
        break;
    end
    all_lines{end+1} = line;
end
fclose(fid);

% --- Locate the data header line ("f(hz), t = ...") ---
data_start_line = -1;
for i = 1:numel(all_lines)
    if strncmp(all_lines{i}, 'f(hz)', 5)
        data_start_line = i;
        break;
    end
end
if data_start_line < 0
    error('Could not find data header line');
end

header_lines = all_lines(1:data_start_line-1);
fprintf('%s\n', header_lines{:});

% --- Find the GyroAccumulator block ---
filter_line = -1;
for i = 1:numel(header_lines)
    if strncmp(header_lines{i}, 'GyroAccumulator', 15)
        filter_line = i;
        break;
    end
end
if filter_line < 0
    error('Could not find GyroAccumulator block');
end

sample_rate = sscanf(header_lines{filter_line+1}, 'Sample rate: %f Hz');
output_rate = sscanf(header_lines{filter_line+2}, 'Output rate: %f Hz');
N           = round(sample_rate / output_rate);

% --- Parse time stamps from the data header line ---
parts = strsplit(all_lines{data_start_line}, ',');
time = zeros(1, numel(parts) - 1);
for i = 2:numel(parts)
    time(i-1) = sscanf(strtrim(parts{i}), 't = %f');
end

% --- Read numeric data (dlmread row index is 0-based) ---
% data_start_line is 1-based; dlmread(file, ',', R, C) skips R rows (0-based).
% Passing data_start_line skips the f(hz) header and lands on the first data row.
data   = dlmread(filename, ',', data_start_line, 0);
freq   = data(:,1);
output = data(:,2:end);

% --- Brute-force amplitude and phase via least-squares sine fit ---
% Fit only the last 100 output samples (steady state).
% Model: Y = A*sin(2*pi*f*t) + B*cos(2*pi*f*t)  (no DC term)
% https://math.stackexchange.com/questions/3926007
num_sweep          = numel(freq);
amplitude          = zeros(num_sweep, 1);
phase              = zeros(num_sweep, 1);
num_best_fit_pts   = 100;
best_fit_idx       = (numel(time) - num_best_fit_pts + 1):numel(time);

for i = 1:num_sweep
    % At exact multiples of output_rate/2 the sin basis is zero for every
    % output sample, making the least-squares matrix rank-deficient.
    % Skip those points and leave them as NaN.
    if mod(freq(i), output_rate/2) < 0.5
        amplitude(i) = NaN;
        phase(i)     = NaN;
        continue;
    end
    t_fit = time(best_fit_idx);
    X = [sin(t_fit * 2*pi*freq(i));
         cos(t_fit * 2*pi*freq(i))];
    Y     = output(i, best_fit_idx);
    Z_fit = Y / X;
    amplitude(i) = sqrt(Z_fit(1)^2 + Z_fit(2)^2);
    phase(i)     = atan2(Z_fit(2), Z_fit(1)) * 180/pi;
end

% --- Evaluate H(z) on the unit circle at the IMU sample rate ---
% z = e^(j*2*pi*f/sample_rate)
% The causal FIR accumulates N samples after the block-start timestamp, so
% the filter output sits (N-1) IMU samples later than the brute-force time
% reference.  Multiply by Z^(N-1) to shift the phase into the same frame.
Z             = exp(1i*pi*(freq / (sample_rate*0.5)));
H             = (1/(2*sample_rate)) .* (1 + Z.^-1) .* (1 - Z.^(-N)) ./ (1 - Z.^-1);
dc_gain        = N / sample_rate;   % lim z->1 of H(z) = 1/output_rate
calc_amplitude = abs(H) / dc_gain;
amplitude      = amplitude / dc_gain;
calc_phase     = atan2(imag(H .* Z.^(N-1)), real(H .* Z.^(N-1))) * 180/pi;

% --- Report errors (degenerate frequencies excluded) ---
amp_error   = abs(amplitude - calc_amplitude);
fprintf('Amplitude error - max: %g  mean: %g\n', max(amp_error(~isnan(amp_error))),   mean(amp_error(~isnan(amp_error))));
phase_error = abs(mod(phase - calc_phase + 180, 360) - 180);
fprintf('Phase error     - max: %g  mean: %g\n', max(phase_error(~isnan(phase_error))), mean(phase_error(~isnan(phase_error))));

% --- Bode plot ---
figure;

subplot(2,1,1);
hold on;
plot(freq, amplitude);
plot(freq, calc_amplitude);
ylabel('Magnitude (normalised to DC)');
legend('Brute force', 'Transfer function');
title(sprintf('Gyro Accumulator  f_{IMU}=%g Hz  f_{out}=%g Hz  N=%d', ...
      sample_rate, output_rate, N));

subplot(2,1,2);
hold on;
plot(freq, phase);
plot(freq, calc_phase);
xlabel('Frequency (Hz)');
ylabel('Phase (deg)');
