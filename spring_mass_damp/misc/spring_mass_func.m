function [A,B,C,D] = spring_mass_func(m,k,b,Ts)
A = [0 1; -k/m -b/m];
B = [0 1/m]';
C = [1 0];
D = [0];
end
