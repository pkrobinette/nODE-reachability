%% System Identification for the mass spring system using classic techniques.
% TO DO: Verify Equations with reachability??
clear; clc;

%% Initialize Variables
m = 10;     % mass
k = 0.8;    % spring constant
c = 0.5;    % damper constant
Ts = 0.05;  % sampling time
n = 1e-5;   % noise coefficient

% Can either use transfer function model or integral model.
data = sim('spring_mass_damp_tf'); % simulate the mass spring damper model
%data = sim('spring_mass_damp');


%% Estimate model and parameters

s_data = iddata(data.y, data.u); % organize data
np = 2; % number of poles
nz = 0; % number of zeros
sys_test = tfest(s_data, np, nz); % estimate transfer function

% Obtain model parameters and associated uncertainty data
pvec = getpvec(sys_test);

% Compare data to estimated function
figure
compare(s_data, sys_test)


%% Save workspace
save('spring_ws')


%% Estimate Parameters-- a different way
% nb = 0; % Zeros
% na = 2; % Poles
% 
% [b, a] = stmcb(data.y, data.u, nb, na);
% 
% model_output = filter(b, a, data.u);
% 
% exp = iddata(data.y, data.u);
% model = iddata(model_output,data.u);
% 
% figure(1)
% compare(exp, model)

% est_b = a(2)*m;
% est_k = a(3)*m;

