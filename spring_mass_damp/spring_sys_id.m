% System Identificaiton of the spring mass damper model
% TO DO: 1. Figure out why not working, add PWM for force and add noise,
% check through: https://www.mathworks.com/help/ident/gs/about-system-identification.html

%% Clear
clear;
clc;

%% Load Data
load('training_data/spring_training4')
% data = iddata(out.x, out.F, Ts, 'Name', 'Spring Mass Model');
data = iddata([zeros(5, 1); out.x], [zeros(5,1); out.F]); % zeros(5,1) is to appease warning
% data.InputName = 'Force';
% data.InputUnit = 'N';
% data.OutputName = {'Position'};
% data.OutputUnit = {'m'};
% data.Tstart = 0;
% data.TimeUnit = 's';
%%
% plot(data)

%% idgrey variables
odefunc = 'spring_mass_func';
m = 5;
k = 300;
b = 10;

parameters = {m k b};  % parameters in the spring mass damper model
fcn_type = 'c'; % continuous function
opt_args = {};

init_sys = idgrey(odefunc, parameters, fcn_type);
% init_sys = idgrey(odefunc, parameters, fcn_type, opt_args, 0);

%Specify lower bounds for mass
init_sys.Structure.Parameters(1).Minimum = 0;

%% Greyest sys id
sys = greyest(data, init_sys, greyestOptions('EnforceStability',true));
% sys.algorithm.SimulationOptions.Solver = 'ode45';
% sys.algorithm.SimulationOptions.MaxStep = 1e-1;
% sys.algorithm.SimulationOptions.InitialStep = 1e-4;
% sys.SimulationOptions.AbsTol = 1e-6;
% sys.SimulationOptions.RelTol = 1e-5;

% opt = compareOptions('InitialCondition', 'zero');
% opt = greyestOptions('EnforceStability','true', 'InitialState', 'estimate', 'DisturbanceModel', 'none');
% opt = greyestOptions('EnforceStability', 'true');

compare(data,sys)
