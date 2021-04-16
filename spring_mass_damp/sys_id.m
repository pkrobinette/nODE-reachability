% par = [3; 50; 2];
% aux = {};
% T = 0;
% m = idgrey('spring_sys', par, 'c');
% 
% load('training_data/spring_training1')
% 
% m_est = greyest(data.x, m);

% FileName = 'spring';
% Order = [1 2 1];
% Parameters = [3; 50; 2];
% InitialStates = [
% % Ts = 0;

%  something with iddata for time-domain estimation



% Make ts a variable and save it for later.

% data = iddata(out.x, out.F, 0.005z0;
%  init_sys = idgrey(odefunction, parameters, type = 'c', optional_args,
%  Ts)

%% Clear
clear;
clc;

%% Load Data
load('training_data/spring_training4')
% data = iddata(out.x, out.F, Ts, 'Name', 'Spring Mass Model');
data = iddata(out.x, out.F);
% data.InputName = 'Force';
% data.InputUnit = 'N';
% data.OutputName = {'Position'};
% data.OutputUnit = {'m'};
% data.Tstart = 0;
% data.TimeUnit = 's';
%%
plot(data)

%% idgrey variables
odefunc = 'spring_mass_sys';
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

% % opt = compareOptions('InitialCondition', 'zero');
% opt = greyestOptions('EnforceStability','true', 'InitialState', 'estimate', 'DisturbanceModel', 'none');
% opt = greyestOptions('EnforceStability', 'true');

compare(data,sys)
