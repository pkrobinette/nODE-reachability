% State space model for the spring mass damper system
% m: mass (kg)
% k: spring constant (N/m)
% b: damping constant (Ns/m)
% F: input force (N)
% 
% A = [0 1; -k/m -b/m];
% B = [0 1/m]';
% C = [1 0];
% D = [0];

clear

m = 10;
k = 400;
b = 3;
F = 1;
x0 = 4;
Ts = 0;

out = sim('spring_mass_damp');

save('training_data/spring_training5')

