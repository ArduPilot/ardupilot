addpath(genpath('../../MATLAB'))

% init the globalr varables required by the heli SITL example

global A
global B

A=zeros(6,6); %#ok<*PREALL>
B=zeros(6,7);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initialize higher order model variables
DCM_0 = zeros(3,3);
ic_vel = zeros(3,1);
ic_angrate = zeros(3,1);
ic_controls = zeros(4,1);
ic_euler_ang = zeros(3,1);

% direction IC's
h_0=0; h=0;
North_0=0;%
East_0=0;
Psi = 353*pi/180.0;

%Derivatives accountfor mass 
M=1;

Xu=-0.02;     %   barn door
Zw=-2.0;
Mq=-2.0;
Mu=0.001;
Yv=-0.4; 
Lp=-6.0;
Nr=-8.0;
Lv=-0.02;



%     [u     v      w     p     q      r]'

A = [   Xu    0     0     0     0      0;...
        0     Yv    0     0     0      0;...
        0     0     Zw    0     0      0;...
        0     Lv    0     Lp    0      0;...
        Mu    0     0     0     Mq     0;...
        0     0     0     0     0      Nr];


%      lat long coll ped '

Zcoll=-60;
Mlong = 1;
Llat=1;
Nped=2;		 
B =   [ 0     0     0     0;...
        0     0     0     0;...
        0     0    Zcoll  0 ;...
        Llat  0     0     0 ;...
        0    Mlong  0     0 ;...
        0     0     0    Nped ];

max_saved=300000;   % limits no. of data points to the last max_saved

%%  Re-order rows of  A for aerospace blk based sim - columns to be ordered in u.
% Pare A down for Forces only 
A_Fxyz_uvw_pqr=A([1 2 3],[1 2 3 4 5 6]); 
% Pare A down for Moments only
A_Mxyz_uvw_pqr=A([4 5 6],[1 2 3 4 5 6]); 
              
%%  B rows = X,Y,Z,L,M,N,      B cols  =  u=[lat long coll ped]
% Pare B down for Forces only
B_Fxyz_u_cont=B([1 2 3],[1 2 3 4]); 
% Pare B down for Moments only
B_Mxyz_u_cont=B([4 5 6],[1 2 3 4]); 

ic_u=0.0000; ic_v=0.0000;  ic_w=0.0000; zPsi=306*pi/180; % Points down RWY32

%ic_u=0.00001; ic_v=0.00001;  ic_w=0.00001; zPsi=306*pi/180; % Points down RWY32
ic_U=sqrt(ic_u^2+ic_v^2+ic_w^2);


%% WIND MODEL - NOTE ; SET Vw = 0 @ h_ic,  Vw units in braces are KTS
h_deck=21;  h_island=h_deck+130;
h_contact=h_deck; % h_contact is used in landing gear model (same for fixed and rotary model)

%% TURBULENCE
W_GUST=.8; % BW in rad/sec
K_gust=0 ;  % 80; % Gust Gain Factor
Th_wind_limit=40;  % Limits on wind swing in deg
hw_gust=[0 1 h_island h_island+1 5000];  % 5 break points
K_gust_factor=[1.1 1 .5 0.25 0];
%% WIND
% hw=[0, h_deck+1, h_deck+1, 150 151 20000];   % FOR SHIP.. 6 BREAK POINTS
hw=[0 200 5000 10000 15000 20000];   % 6 BREAK POINTS, MONO INCREASING
Vw=15*[1 1 1 1 1 1];       % wind magnitude in fps
Dir_w=0*[1 1 1 1 1 1];    % wind direction in deg

Ftrim_grav=-32.0757;
Roll_ic=0;  Pit_ic=0;  

%Make the trim direction cosine matrix 
phi_0 = 3; 
tht_0 = Pit_ic;
psi_0 = Psi*180/pi;

cphi = cosd(phi_0);
sphi = sind(phi_0);
ctht = cosd(tht_0);
stht = sind(tht_0);
cpsi = cosd(psi_0);
spsi = sind(psi_0);

DCM_0 = [1 0 0;
    0 cphi sphi;
    0 -sphi cphi] * ...
    [ctht 0 -stht;
    0     1  0;
    stht  0  ctht]*...
    [cpsi spsi 0;
    -spsi cpsi 0
    0     0    1];

%% A short run of the sim to sample cockpit the current control positions 
U_CONT_O=[0 0 0 0]; 


Ts=2000;     % set Ts for real time sim run
disp('Init Loaded')