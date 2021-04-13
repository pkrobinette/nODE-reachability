% Load data from simulations of buck_conv2.slx
% Data:
% - out.t: time
% - out.vars(:,1): Vin, the input voltage to the system
% - out.vars(:,2): V_L, the voltage across the inductor
% - out.vars(:,3): I_L, the current across the inductor
% - out.vars(:,4): V_out, the voltage across the load
% - out.vars(:,5): I_out, the current across the load

clear

load("training1.mat")
figure(1)
plot(out.t, out.vars(:,2))

load("training2.mat")
figure(2)
plot(out.t, out.vars(:,2))