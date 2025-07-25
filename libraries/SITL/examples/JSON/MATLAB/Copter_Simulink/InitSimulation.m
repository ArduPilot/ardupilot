clear; clc; close all;
addpath(genpath('../../MATLAB'))

%  1. motor is located top right corner of drone and motor number increase 
%  consecutevily clockwise.

% simulation time step
dt = 0.001;

% load frame parameters
modelParameters;

% run simulation
sim('quadrotor.slx')