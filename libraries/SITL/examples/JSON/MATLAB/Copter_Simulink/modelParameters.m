% Information about the model properties can be found on following sites
% https://flyeval.com/index.html
% https://flyeval.com/js/MathModelDocEn.pdf

% Quadcopter-250mm-0.6kg-15min-EMAX model is used. Only Cr coefficient is inresed by 50%

gravity = 9.81;

if 1
load('Quadcopter-250mm-0.6kg-15min-EMAX.mat');
% load('Quadcopter-450mm-1.5kg-7min-DJI.mat');
else
vehicle.mass = 1.5;
vehicle.Inertia = diag([1.745e-2, 1.745e-2, 3.175e-2]);
vehicle.distance2motor = 0.255;
vehicle.lx = sind(45)*vehicle.distance2motor;
vehicle.ly = sind(45)*vehicle.distance2motor;
vehicle.Ct = 1.105e-5;
vehicle.Cm = 1.489e-7;
vehicle.wHover = sqrt(vehicle.mass*gravity/(4*vehicle.Ct));

vehicle.Cr = 646.53;
vehicle.wb = 324.68; % not used
vehicle.InertiaMotor = 9.90e-5;
vehicle.MotorTimeConstant = 0.0136;

vehicle.Cd = 6.579e-2;
vehicle.Cdm = 9.012e-3;  
% save('Quadcopter-450mm-1.5kg-7min-DJI.mat','vehicle')
end
thrust2weight_ratio = vehicle.Cr^2*vehicle.Ct*4/(vehicle.mass*gravity);

% motor rotational speed to torque and thrust 
controlEffec.G1 = ...
[-vehicle.ly*vehicle.Ct, -vehicle.ly*vehicle.Ct, vehicle.ly*vehicle.Ct, vehicle.ly*vehicle.Ct;
 vehicle.lx*vehicle.Ct, -vehicle.lx*vehicle.Ct, -vehicle.lx*vehicle.Ct, vehicle.lx*vehicle.Ct;
 vehicle.Cm, -vehicle.Cm, vehicle.Cm, -vehicle.Cm;
 -vehicle.Ct, -vehicle.Ct, -vehicle.Ct, -vehicle.Ct];

% motor rotational speed to torque and thrust 
controlEffec.G2 = ...
[0, 0, 0, 0;
 0, 0, 0, 0;
 vehicle.InertiaMotor, -vehicle.InertiaMotor, vehicle.InertiaMotor, -vehicle.InertiaMotor;
 0, 0, 0, 0];

controlEffec.G3 = vehicle.InertiaMotor*...
[ 1, -1,  1, -1;
 -1,  1, -1,  1;
  0,  0,  0,  0;
  0,  0,  0,  0];