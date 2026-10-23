close all
clear
clc

% Script for plotting outputs of SlewLimiter examples

files = dir('*.csv');
for i = numel(files):-1:1
    data = readmatrix(files(i).name);

    run{i}.time = data(:,1);
    run{i}.input = data(:,2);
    run{i}.slew = data(:,3);
    run{i}.mod = data(:,4);

    legend_val{i} = strrep(files(i).name, '_' ,'\_');
end

figure
tiledlayout(3,1)

nexttile
hold all
for i = 1:numel(run)
    plot(run{i}.time, run{i}.input)
end
ylabel('Input')

nexttile
hold all
for i = 1:numel(run)
    plot(run{i}.time, run{i}.slew)
end
legend(legend_val,'Location','eastoutside')
ylabel('Slew rate')

nexttile
hold all
for i = 1:numel(run)
    plot(run{i}.time, run{i}.mod)
end
xlabel('Time (s)')
ylabel('mod')
