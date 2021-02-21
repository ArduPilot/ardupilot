clc
clear all
close all

%{
Very basic Velocity Prediction Program, this duplicates the physics used in the sailboat SITL model. It can be used to generate polar plots to check
STIL physics is sensible. 
 
In the future it is hoped this program can be used as a comparison for a built in polar generator that is yet to be added to the ArduRover code.
Alternatively a drag polar generated here can be used directly in the code to select optimal sailing angles. 
 
Sail lift, drag and Hull drag not considered properly, boat heel not considered. The polar generated is however realistic for a 'generic' sailboat.   
%}

sail_aoa_in =   [0,    10,   20,   30,   40,   50,   60,   70,   80,   90,    100,   110,   120,   130,   140,   150,   160,   170,  179.9]; % deg
lift_curve_in = [0.00, 0.50, 1.00, 1.10, 0.95, 0.75, 0.60, 0.40, 0.20, 0.00, -0.20, -0.40, -0.60, -0.75, -0.95, -1.10, -1.00, -0.50, 0];
drag_curve_in = [0.10, 0.10, 0.20, 0.40, 0.80, 1.20, 1.50, 1.70, 1.90, 1.95,  1.90,  1.70,  1.50,  1.20,  0.80,  0.40,  0.20,  0.10, 0.10];


min_sail_angle = 0; % deg
max_sail_angle = 90; % deg

% intpolate to more detailed curves
sail_aoa = sail_aoa_in(1):0.1:sail_aoa_in(end)-0.1;
lift_curve = interp1(sail_aoa_in, lift_curve_in, sail_aoa);
drag_curve = interp1(sail_aoa_in, drag_curve_in, sail_aoa);

% Plot lift curve
figure 
subplot(2,1,1)
hold all
title('Lift and Drag Curves')
scatter(sail_aoa_in,lift_curve_in,'*','r')
plot(sail_aoa,lift_curve,'r')
ylabel('Cl')

subplot(2,1,2)
hold all
scatter(sail_aoa_in,drag_curve_in,'*','b')
plot(sail_aoa,drag_curve,'b')
xlabel('AoA (deg)')
ylabel('Cd')

%% VPP
% Do for range of sailing angles and wind speeds, increse speed until foward force equals zero
wind_speed = 3:2:15; % m/s
heading = 0:1:179; % deg
speed_step = 0.01; % m/s

for n = 1:length(wind_speed)
    for m = 1:length(heading)
        
        thrust = 1;
        boat_speed{n,m}(1) = 0;
        j = 0;
        while thrust > 0 &&  boat_speed{n,m}(end) < 10
            j = j + 1;
            if j == 1
                boat_speed{n,m}(j) = speed_step;
            else
                boat_speed{n,m}(j) = boat_speed{n,m}(j-1) + speed_step;
            end 
            
            % calculate apparent wind in earth-frame (this is the direction the wind is coming from)
            wind_apparent_speed{n,m}(j) = sqrt(wind_speed(n)^2 + boat_speed{n,m}(j)^2 + 2 * wind_speed(n) * boat_speed{n,m}(j) * cosd(heading(m)));
            
            % Aparent wind angle in body frame
            wind_apparent_dir_bf{n,m}(j) = acosd((wind_speed(n) * cosd(heading(m)) + boat_speed{n,m}(j))/wind_apparent_speed{n,m}(j));
                      
            % calculate lift at all angles-of-attack from wind to mainsail           
            % calculate Lift force (perpendicular to wind direction) and Drag force (parallel to wind direction)
            lift = lift_curve *  wind_apparent_speed{n,m}(j);
            drag = drag_curve *  wind_apparent_speed{n,m}(j); 
            
            % rotate lift and drag from wind frame into body frame
            sin_rot = sind(wind_apparent_dir_bf{n,m}(j));
            cos_rot = cosd(wind_apparent_dir_bf{n,m}(j));
            force_fwd = (lift * sin_rot) - (drag * cos_rot);
                        
            % accel in body frame due acceleration from sail and deceleration from hull friction
            hull_drag =  0.5 * boat_speed{n,m}(j)^2;
            thrust_range = force_fwd - hull_drag;
                             
            % angle of sail to boat 
            sail_boat_angle = wind_apparent_dir_bf{n,m}(j) - sail_aoa;
            % discard imposible sail angles
            index = find(sail_boat_angle >  max_sail_angle | sail_boat_angle <  min_sail_angle);
            thrust_range(index) = NaN;
                        
            % select sail angle that gives best thrust
            index = find(thrust_range == max(thrust_range),1);
            
            if ~isempty(index) 
                thrust = thrust_range(index);
                sail_force{n,m}(j) = force_fwd(index);
                sail_angle_trim{n,m}(j) = sail_boat_angle(index);
                hull_drag_trim{n,m}(j) = hull_drag;
            else
                thrust = 0;
            end
        end
        
        max_boat_speed(n,m) =  boat_speed{n,m}(j);
        % Set minumum values to NaN to make plot neater
        if  max_boat_speed(n,m) == speed_step
             max_boat_speed(n,m) = NaN;
        end
    end
end


%% Polar plot 
figure 
for n = 1:length(wind_speed)
    polarplot(deg2rad(heading),max_boat_speed(n,:))
    hold on
    legend_val{n} = sprintf('%g m/s',wind_speed(n));
end
ax = gca;
d = ax.ThetaDir;
ax.ThetaDir = 'clockwise';
ax.ThetaZeroLocation = 'top';
legend(legend_val)
title('Polar Plot, boat speed (m/s) vs heading (deg)')


%% Plot sail thrust and hull drag for a given heading
plot_heading = 50;
plot_wind_speed = 5;

% convet from values to indexs
plot_heading = find(heading == plot_heading);
plot_wind_speed = find(wind_speed == plot_wind_speed);

if isempty(plot_heading) || isempty(plot_wind_speed)
    fprintf('Plot headings and or windspeed not within the range caulated')
    return
end

figure
subplot(2,1,1)
hold all
title(sprintf('Thrust vs drag for heading of %g deg and wind speed of %g m/s',heading(plot_heading),wind_speed(plot_wind_speed)))
yyaxis left
plot(boat_speed{plot_wind_speed,plot_heading},wind_apparent_speed{plot_wind_speed,plot_heading})
ylabel('Apparent wind speed (m/s)')
yyaxis right
plot(boat_speed{plot_wind_speed,plot_heading},wind_apparent_dir_bf{plot_wind_speed,plot_heading})
ylabel('Apparent wind direction (deg)')


subplot(2,1,2)
hold all
plot(boat_speed{plot_wind_speed,plot_heading},sail_force{plot_wind_speed,plot_heading})
plot(boat_speed{plot_wind_speed,plot_heading},hull_drag_trim{plot_wind_speed,plot_heading})

legend('Sail Forwards force','Hull drag','Location','northwest')
ylabel('Force (N)')
xlabel('Boat speed (m/s)')
