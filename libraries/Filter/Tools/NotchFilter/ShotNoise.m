close all
clear
clc

rate = single(2000);
dT = 1/rate;

time = dT:dT:1000;
samples = single(numel(time));

target_freq = 200;

signal = sin(time*2*pi*target_freq) * 10 + randn(1,samples) * 3;

center_move_freq = single(1/3);

steps = time(end) / center_move_freq;
center = target_freq + randn(1,steps)*50;
center = repmat(center, [ceil(samples/steps),1]);
center = center(:);

center = single(center);

bandwidth = single(5);
attenuation = single(40);

notch1 = NotchFilter;
notch2 = NotchFilter2;


[A, Q] = notch1.calculate_A_and_Q(center(1), bandwidth, attenuation);

center_update_freq = 400;

output1 = zeros(size(signal),'single');
output2 = zeros(size(signal),'single');
notch_center = zeros(size(signal),'single');
notch_factor = zeros([numel(signal),5],'single');
last_center_update = 0;
for i = 1:numel(signal)

    if (last_center_update == 0) || ((time(i) - last_center_update) > (1/center_update_freq))
        % centers are only moved at loop rate not sample rate
        notch1 = notch1.init_with_A_and_Q(rate, center(i), A, Q);
        notch2 = notch2.init_with_A_and_Q(rate, center(i), A, Q);
        last_center_update = time(i);
    end

    notch_center(i) = notch1.center_freq_hz;

    [notch1, output1(i)] = notch1.apply(signal(i));
    [notch2, output2(i)] = notch2.apply(signal(i));
end

max_input = max(abs(signal));
max_output1 = max(abs(output1));
max_output2 = max(abs(output2));

figure
tiledlayout(2,1)
ax{1} = nexttile;
hold all
plot(time,signal)
plot(time,output1)
plot(time,output2)
xlabel("time (s)")
ylabel("amplitude")
legend('input','current output','new output','location','eastoutside')
ylim([-1,1]*max([max_input,max_output1,max_output2])*1.05)

ax{2} = nexttile;
hold all
plot(time,center(1:numel(time)))
plot(time,notch_center)
yline(target_freq,'--k')
xlabel("time (s)")
ylabel("notch center freq")
legend('target center','slewed center','signal freq','location','eastoutside')


fprintf('Max input: %0.2f\n',max_input)
fprintf('Max output 1: %0.2f\n',max_output1)
fprintf('Max output 2: %0.2f\n',max_output2)

linkaxes([ax{:}],'x')


