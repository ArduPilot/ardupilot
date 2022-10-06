function [sigOut] = demodYawAngle(sigIn)
% Inverse the modulo of 360° operation used on the yaw angle signals

% Output signal
sigOut = zeros(length(sigIn),1);

% Calcualte the differences between samples
sigDiff = diff(sigIn);

% If difference is larger than 2*pi, safe last sample as offset for
% following signals
offset = 0;
skip = 0; % Var to skip the signal sample directly after abrupt angle change
for i = 1:length(sigIn)
    sigOut(i) = offset + sigIn(i+skip);
    if i < length(sigDiff) && i > 1
        % Reset prevent Adding
        if abs(sigDiff(i)) < 90
            skip = 0;
        % Add to offset, if switch happened from 2pi to 0
        elseif sigDiff(i) <= -90 && ~skip
            offset = offset + 360;%(sigIn(i-2)+sigIn(i-1))/2; % Smooth signal values since they are oscillating at jumps
            skip = 1;
            % Subtract from offset, if switch happened from 0 tp 2 pi
        elseif sigDiff(i) > 90 && ~skip
            offset = offset - 360;%(sigIn(i+2)+sigIn(i+1))/2; % Smooth signal values since they are oscillating at jumps
            skip = 1;
        end
    end
end

% Apply moving average filter to smooth out sample value jumps
steps = 10;
coeffFilt = ones(1, steps)/steps;
% sigOut = filter(coeffFilt, 1, sigOut);
sigOut = movmean(sigOut, 9);
end

