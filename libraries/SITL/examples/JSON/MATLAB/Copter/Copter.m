clc
clear
close all

% ArduCopter SITL example vehicle
% This sets up the vehicle properties that are used in the simulation
% simple 450 size x quad based on Hexsoon

% setup the motors

% locations (xyz) in m
motor(1).location = [[sind(45),cosd(45)]*450*0.5,0] * 0.001; % front right
motor(2).location = [[sind(135),cosd(135)]*450*0.5,0] * 0.001; % rear right
motor(3).location = [[sind(225),cosd(225)]*450*0.5,0] * 0.001; % rear left
motor(4).location = [[sind(315),cosd(315)]*450*0.5,0] * 0.001; % front left

% PWM output to use
motor(1).channel = 1;
motor(2).channel = 4;
motor(3).channel = 2;
motor(4).channel = 3;

% rotation direction: 1 = cw, -1 = ccw
motor(1).direction = -1;
motor(2).direction = 1;
motor(3).direction = -1;
motor(4).direction = 1;

% motor properties
electrical.kv = 880; % (rpm/volt)
electrical.no_load_current = [0.7,10]; % (A) @ (V)
electrical.resistance = 0.115; % (ohms)

% ESC properties
esc.resistance = 0.01; % (ohms)

% Propeller properties
prop.diameter = 245 * 0.001; % (m)
prop.pitch = 114.3 * 0.001; % (m)
prop.num_blades = 2;
prop.PConst = 1.13;
prop.TConst = 1;
prop.mass = 12.5*0.001; % (kg) (only used for inertia)
prop.inertia = (1/12)*prop.mass*prop.diameter^2; % rotational inertia (kgm^2) (rod about center)

% assign properties to motors
for i = 1:4
    motor(i).electrical = electrical;
    motor(i).esc = esc;
    motor(i).prop = prop;
end

% Setup battery
battery.voltage = 4*4.2; % (volts)
battery.resistance = 0.0034; % (ohms)
battery.capacity = 5.2; % (ah)

% Add all to vehicle
copter.motors = motor;
copter.battery = battery;
copter.mass = 2; % (kg)
inertia = (2/5) * copter.mass * (0.45*0.2)^2; % (sphere)
copter.inertia = diag(ones(3,1)*inertia); % rotational inertia matrix (kgm^2) 
copter.cd = [0.5;0.5;0.5];
copter.cd_ref_area = [1;1;1] * pi * (0.45*0.5)^2;

save('Hexsoon','copter')

% Plot motor curves
% http://www.bavaria-direct.co.za/constants/
% http://www.stefanv.com/rcstuff/qf200204.html
% Some calculators estimate heat and increase resistance with temp
% But then we have to estimate the power dissipation
% Max power for plot only
max_power = 260;
battery.voltage = battery.voltage * 0.50;

Kt = 1/(electrical.kv * ((2*pi)/60) ); % Convert Kv to rads/second

% plot the current from 0 to max power
amps = 0:0.1:max_power/battery.voltage;
power_in = amps * battery.voltage;

% voltage drop due to copper and esc
copper_drop = amps * electrical.resistance; 
esc_drop = amps * esc.resistance;

ideal_voltage = battery.voltage - copper_drop - esc_drop;
power_out = ideal_voltage .* (amps - electrical.no_load_current(1));
efficiency = power_out ./ power_in;

torque = Kt * amps;
rpm = ideal_voltage * electrical.kv;

% Plot motor characteristics
figure('name',sprintf('motor characteristics at %0.2f volts',battery.voltage))
subplot(2,2,1)
hold all
title('RPM')
plot(amps,rpm)
xlabel('Current (A)')
ylabel('RPM')
xlim([0,amps(end)])


subplot(2,2,2)
hold all
title('torque')
plot(amps,torque)
xlabel('Current (A)')
ylabel('torque (NM)')
xlim([0,amps(end)])

subplot(2,2,3)
hold all
title('power')
plot(amps,power_in)
plot(amps,power_out)
xlabel('Current (A)')
ylabel('power (W)')
ylim([0,inf])
xlim([0,amps(end)])
legend('Power in','Power out','location','northwest')

subplot(2,2,4)
hold all
title('efficiency')
plot(amps,efficiency)
xlabel('Current (A)')
ylabel('efficiency (%)')
ylim([0,inf])
xlim([0,amps(end)])
