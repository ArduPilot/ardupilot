clc
clear
close all

% generate input files with: ./build/linux/examples/AP_Motors_test s >> output.csv

AP_MOTORS_MATRIX_YAW_FACTOR_CW =  -1;
AP_MOTORS_MATRIX_YAW_FACTOR_CCW = 1;

% Set matching frame type and class
%{
frame_definition = [ % quad plus
      90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2;
     -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4;
       0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1;
     180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3;];
%}
%{
frame_definition = [ % quad x
       45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1;
     -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3;
      -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4;
      135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2;];
%}
%{
frame_definition = [ % hex x
      90, AP_MOTORS_MATRIX_YAW_FACTOR_CW    2;
     -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5;
     -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6;
     150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3;
      30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1;
    -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4];
%}
%{
frame_definition = [ % octo X
      22.5,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1;
    -157.5,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5;
      67.5,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2;
     157.5,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4;
     -22.5,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8;
    -112.5,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  6;
     -67.5,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7;
     112.5,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3];
%}
%%{
frame_definition = [ % octo quad x
     45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1;
    -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7;
   -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5;
    135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3;
    -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8;
     45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2;
    135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4;
   -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6];
%}
%{
frame_definition = [ % octo quad x
     30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   1;
     30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    2;
     90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    3;
     90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   4;
    150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   5;
    150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    6;
   -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    7;
   -150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   8;
    -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   9;
    -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10;
    -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   11;
    -30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  12];
%}

num_motors = size(frame_definition,1);

% roll, pitch, yaw, throttle
motor_factors = [cosd(frame_definition(:,1) + 90), cosd(frame_definition(:,1)), frame_definition(:,2), ones(num_motors,1)];
motor_factors = (motor_factors ./ max(abs(motor_factors))) .* [0.5, 0.5, 0.5, 1];

files = dir('*.csv');
if isempty(files)
	error('no .csv files found')
end

for i = numel(files):-1:1
    results(i).raw = readmatrix(files(i).name);
    if size(results(i).raw,2) ~= (num_motors*2) + 9
        error('%s - found wrong number of motors', files(i).name)
    end
    results(i).name = extractBefore(files(i).name,'.');
    
    results(i).roll_in = unique(results(i).raw(:,1));
    results(i).pitch_in = unique(results(i).raw(:,2));
    results(i).yaw_in = unique(results(i).raw(:,3));
    results(i).throttle_in = unique(results(i).raw(:,4));
    
    results(i).norm_out = nan(numel(results(i).roll_in),numel(results(i).pitch_in),numel(results(i).yaw_in),numel(results(i).throttle_in),num_motors);
    results(i).achieved_output = nan(numel(results(i).roll_in),numel(results(i).pitch_in),numel(results(i).yaw_in),numel(results(i).throttle_in),4);
    results(i).error = nan(numel(results(i).roll_in),numel(results(i).pitch_in),numel(results(i).yaw_in),numel(results(i).throttle_in),4);
    for j = 1:size(results(i).raw,1)
        roll_index = find(results(i).roll_in == results(i).raw(j,1));
        pitch_index = find(results(i).pitch_in == results(i).raw(j,2));
        yaw_index = find(results(i).yaw_in == results(i).raw(j,3));
        throttle_index = find(results(i).throttle_in == results(i).raw(j,4));
        if numel(roll_index) ~= 1 || numel(pitch_index) ~= 1 || numel(yaw_index) ~= 1 || numel(throttle_index) ~= 1
            error('%s - indeing issue', files(i).name)
        end
        results(i).norm_out(roll_index,pitch_index,yaw_index,throttle_index,:) = results(i).raw(j,num_motors+4+(1:num_motors));
        results(i).achieved_output(roll_index,pitch_index,yaw_index,throttle_index,:) = squeeze(results(i).norm_out(roll_index,pitch_index,yaw_index,throttle_index,:))' / motor_factors';
        results(i).error(roll_index,pitch_index,yaw_index,throttle_index,:) = abs(squeeze(results(i).achieved_output(roll_index,pitch_index,yaw_index,throttle_index,:)) - results(i).raw(j,1:4)');
    end    
end

if  any(abs(diff([results(:).roll_in],[],2)) > 10^-6,'all') ||...
    any(abs(diff([results(:).pitch_in],[],2)) > 10^-6,'all') ||...
    any(abs(diff([results(:).yaw_in],[],2)) > 10^-6,'all') ||...
    any(abs(diff([results(:).throttle_in],[],2)) > 10^-6,'all')
    error('inputs do not match')
end

[roll_in,pitch_in] = meshgrid(results(1).roll_in,results(1).pitch_in);

yaw_in = results(1).yaw_in;
throttle_in = results(1).throttle_in;

yaw_index =  ceil(numel(yaw_in)/2);
throttle_index = ceil(numel(throttle_in)/2);

[yaw_in_m, throttle_in_m] = meshgrid(yaw_in, throttle_in);



%% Setup plots
figure
if num_motors <= 10
    tiledlayout(2, num_motors / 2)
else
    tiledlayout(4, num_motors / 4)
end
for i = num_motors:-1:1
    axis1(i) = nexttile(i);
    hold all
    title(sprintf('motor %i',i))
    xlabel('roll in')
    ylabel('pitch in')
    zlim([-0.01,1.01])
    view(3);
end

figure
tiledlayout(2, 2)
titles = {'roll','pitch','yaw','throttle'};
for i = 4:-1:1
    axis2(i) = nexttile(i);
    hold all
    
    if i == 1
        surf(axis2(i),roll_in,pitch_in,roll_in,'EdgeColor','none','FaceColor','k','FaceAlpha',0.25);
    elseif i == 2
        surf(axis2(i),roll_in,pitch_in,pitch_in,'EdgeColor','none','FaceColor','k','FaceAlpha',0.25);
    elseif i == 3
        yaw_ideal = surf(axis2(i),roll_in,pitch_in,yaw_in(yaw_index)*ones(size(roll_in)),'EdgeColor','none','FaceColor','k','FaceAlpha',0.25);
    elseif i ==4
        throttle_ideal = surf(axis2(i),roll_in,pitch_in,throttle_in(throttle_index)*ones(size(roll_in)),'EdgeColor','none', 'FaceColor','k','FaceAlpha',0.25);
        throttle_max = surf(axis2(i),roll_in,pitch_in,get_throttle_avg_max(throttle_in(throttle_index))*ones(size(roll_in)),'EdgeColor','none', 'FaceColor','c','FaceAlpha',0.25);
        if get_throttle_avg_max(throttle_in(throttle_index)) == throttle_in(throttle_index)
            set(throttle_max,'Visible','off');
        else
            set(throttle_max,'Visible','on');
        end
    end
    title(titles(i))
    xlabel('roll in')
    ylabel('pitch in')
    
    if i < 4
        zlim([-1.01,1.01])
    else
        zlim([-0.01,1.01])
    end
    
    view(3);
end



figure
hold all
title('error')
xlabel('roll in')
ylabel('pitch in')
zlim([0,inf])
view(3);
axis3 = gca;


figure
tiledlayout(2, 2)
for i = 4:-1:1
    axis4(i) = nexttile(i);
    hold all
    title([titles{i},' error'])
    xlabel('roll in')
    ylabel('pitch in')
    zlim([0,inf])
    view(3);
end
    
hlink1 = linkprop([axis1,axis2,axis3,axis4],{'CameraPosition'});


%% Plot outputs
colour = 'rbgy';
for i = 1:numel(results)
    for j = 1:num_motors
        results(i).motor_plots(j) = surf(axis1(j),roll_in,pitch_in,results(i).norm_out(:,:,yaw_index,throttle_index,j)','FaceColor',colour(i),'EdgeColor','k','FaceAlpha',0.5);
    end
    for j = 1:4
        results(i).output_plots(j) = surf(axis2(j),roll_in,pitch_in,results(i).achieved_output(:,:,yaw_index,throttle_index,j)','FaceColor',colour(i),'EdgeColor','k','FaceAlpha',0.5);
    end
    results(i).error_plot = surf(axis3,roll_in,pitch_in,sum(squeeze(results(i).error(:,:,yaw_index,throttle_index,:)),3)','FaceColor',colour(i),'EdgeColor','k','FaceAlpha',0.5);
    for j = 1:4
        results(i).error_plots(j) = surf(axis4(j),roll_in,pitch_in,results(i).error(:,:,yaw_index,throttle_index,j)','FaceColor',colour(i),'EdgeColor','k','FaceAlpha',0.5);
    end
end
legend(axis3,{results(:).name},'location','eastoutside','Interpreter','none')

%% Plot throtte and yaw input plot

figure
hold all

c = zeros(numel(yaw_in_m),3);
c(sub2ind(size(yaw_in_m),throttle_index,yaw_index), :) = [1,0,0];


input_plot = scatter(yaw_in_m(:),throttle_in_m(:),[],c,'filled');

xlabel('Yaw input')
ylabel('Throttle input')

xlim([min(yaw_in_m(:)), max(yaw_in_m(:))]*1.1)
ylim([-0.05, 1.05])

while(true)
    [x, y] = ginput(1);
    
    [~, yaw_index] = min(abs(yaw_in-x));
    [~, throttle_index] = min(abs(throttle_in-y));
    
    c = zeros(numel(yaw_in_m),3);
    c(sub2ind(size(yaw_in_m),throttle_index,yaw_index), :) = [1,0,0];
    
    set(input_plot,'CData',c);
    
    
    for i = 1:numel(results)

        for j = 1:num_motors
            if isvalid(results(i).motor_plots(j))
                set(results(i).motor_plots(j),'ZData',results(i).norm_out(:,:,yaw_index,throttle_index,j)');
            end
        end
        for j = 1:4
            if isvalid(results(i).output_plots(j))
                set(results(i).output_plots(j),'ZData',results(i).achieved_output(:,:,yaw_index,throttle_index,j)');
            end
        end
        if isvalid(results(i).error_plot)
            set(results(i).error_plot,'ZData',sum(squeeze(results(i).error(:,:,yaw_index,throttle_index,:)),3)');
        end
        for j = 1:4
            if isvalid(results(i).error_plots(j))
                set(results(i).error_plots(j),'ZData',results(i).error(:,:,yaw_index,throttle_index,j)');
            end
        end
        
    end
    
   
    
     if isvalid(yaw_ideal)
         set(yaw_ideal,'ZData',yaw_in(yaw_index)*ones(size(roll_in)));
     end
     if isvalid(throttle_ideal) && isvalid(throttle_max)
         set(throttle_ideal,'ZData',throttle_in(throttle_index)*ones(size(roll_in)));
         
         throttle_avg_max = get_throttle_avg_max(throttle_in(throttle_index));
         set(throttle_max,'ZData',throttle_avg_max*ones(size(roll_in)));
         if throttle_avg_max == throttle_in(throttle_index)
             set(throttle_max,'Visible','off');
         else
             set(throttle_max,'Visible','on');
         end
     end
end


%% AP max throttle limit
function throttle_avg_max = get_throttle_avg_max(throttle_in)
    throttle_avg_max = max(throttle_in, 0.5);
end

