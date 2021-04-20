% Generate data for the state space model.
% ma = F - bv - kx
% m: mass (kg)
% k: spring constant (N/m)
% b: damping constant (Ns/m)
% F: input force (N)
% x0: initial x position
% Ts: sampling rate
% 
% A = [0 1; -k/m -b/m];
% B = [0 1/m]';
% C = [1 0];
% D = [0];

clear;

m = 10;
k = 400;
b = 3;
F = 1;
x0 = 4;
Ts = 0;

out = sim('spring_mass_damp');

% indicate path of datafile.
save('training_data/spring_training5')

