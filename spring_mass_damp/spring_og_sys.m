%% System Identification for the Mass Spring Damper System
% TO DO:
% Assess these with bOxes of data -- a reach set.
% Figure out a way to do this
clear; clc; close;

m = 10;     % mass
k = 0.8;    % spring constant
c = 0.5;    % damper constant
Ts = 0.05;  % sampling time
n = 1e-5;   % noise coefficient

%% Simulate Data with Simulink Model
% data = sim('spring_mass_damp');
% t = data.t;
% u = data.u(1:length(t));
% y = data.y(1:length(t));

%% Generate Data with created Model
A = [0 1; -k/m -c/m];
B = [0; 1/m];
C = [1 0];
D = [0];

num = [1/m];
den = [1 c/m k/m];

% can either make state space or transfer function, does not matter
sys_ss = ss(A, B, C, D);
% [num, den] = ss2tf(A, B, C, D);
sys_tf = idtf(num, den);

% generating inputs to the system
[u, t] = gensig("square",500 , 1000, 1);
u = u+1;

% % split into training and testing
% N = 700;
% train_u = u(1:N, 1);
% train_t = t(1:N, 1);
% test_u = u(N:end, 1);
% test_t = t(N:end, 1);
% 
[ss_y, ss_t, ss_x] = lsim(sys_ss, u, t);
[tf_y, tf_t, tf_x] = lsim(sys_tf, u, t);


%% Add noise to the output data
n = 0.05; % noise
noise = n*randn(length(ss_y), 1);
ss_noise = ss_y + noise;
tf_noise = tf_y + noise;

%% sys id state space 
ss_data = iddata(ss_noise, u);
% ss_data = iddata(y, u);
nx = 2; % order
ss_est = ssest(ss_data, nx);

%% Compare ss
figure(1);
compare(ss_data, ss_est);
title("State Space Estimation with " + string(n*100) + "% Added Noise");
legend('Actual', 'Estimated');
grid on;
% saveas(gcf, 'ss_est_'+string(n*100)+'_noise.png');

%% sys id transfer function
tf_data = iddata(tf_noise, u);
% tf_data = iddata(y, u);
np = 2; % number of poles
nz = 0; % number of zeros
tf_est = tfest(tf_data, np, nz);

% Obtain model parameters and associated uncertainty data
pvec = getpvec(tf_est);

%% Compare tf
figure(1);
compare(tf_data, tf_est);
title("Transfer Function Estimation with " + string(n*100) + "% Added Noise");
legend('Actual', 'Estimated');
grid on;
% saveas(gcf, 'tf_est_'+string(n*100)+'_noise.png');

%% sys id matlab tool
% tf_data = iddata(tf_noise, train_u);
% 
% figure(1)
% compare(tf_data, tf1);
% title("Transfer Function Estimation using Matlab Tool with " + string(n*100) + "% Added Noise");
% legend('Actual', 'Estimated');
% grid on;
% saveas(gcf, 'mat_est_tf_' +string(n*100)+'_noise.png');

%% Validate Model

% new inputs into the system
[u,t] = gensig("sine", 500 , 1000, 1);
u = u+1;

%% Simulate models

[act_ss_y, act_ss_t, act_ss_x] = lsim(sys_ss, u, t);
[act_tf_y, act_tf_t, act_tf_x] = lsim(sys_tf, u, t);
[pred_ss_y, pred_ss_t, pred_ss_x] = lsim(ss_est, u, t);
[pred_tf_y, pred_tf_t, pred_tf_x] = lsim(tf_est, u, t);

data_act_ss = iddata(act_ss_y, u);
data_act_tf = iddata(act_tf_y, u);
data_pred_ss = iddata(pred_ss_y, u);
data_pred_tf = iddata(pred_tf_y, u);

close;
figure(1);
compare(data_act_ss, data_pred_ss);
grid on;
title('Evaluation of Actual and Estimated State Space Model');
xlabel('Time (t)');
ylabel('Amplitude');
% saveas(gcf, 'ss_comp.png');

figure(2);
compare(data_act_tf, data_pred_tf);
grid on;
title('Evaluation of Actual and Estimate Transfer Function Model');
xlabel('Time (t)');
ylabel('Amplitude');
% saveas(gcf, 'tf_comp.png');

figure(3);
plot(act_ss_t, act_ss_y);
hold on;
plot(pred_ss_t, pred_ss_y);
grid on;
legend('Actual', 'Predicted');
title('Evaluation of Actual and Estimated State Space Model');
xlabel('Time (t)');
ylabel('Amplitude');
% saveas(gcf, 'ss_comp.png');


% err = immse(act_y, pred_y);
% disp(err)

%% Create acceptable upper and lower bound

% maybe make array
ub = act_y + 0.25;
lb = act_y - 0.25;

%% Plot Data

figure(3);
plot(act_t, ub, 'r');
hold on
plot(act_t, lb, 'r');
hold on
plot(pred_t, pred_y, '-b');

%% Reachabilty -- CORA

% System Dynamics ---------------------------------------------------------

A1 = [0 1; -pvec(3) -pvec(2)];
B1 = [0; pvec(1)];
C1 = [1 0];
D1 = [0];

obj1 = linearSys(A, B, [], C, D);
obj2 = linearSys(A1, B1, [], C1, D1);

% Parameters ---------------------------------------------------------------

% final time, initial set, and uncertain inputs
params.tFinal = 20;
params.R0 = zonotope(interval([2;2], [2.5;2.5])); %% TO DO: Maybe play around with just [2;2.5]
params.U = zonotope(interval(0, 0.5));
% params.U = zonotope([1;-0.5],[0.2,0.5]);

% Specifications ------------------------------

S = interval(10, 12);
spec = specification(S, 'unsafeSet');

% Reachability Settings ---------------------------------------------------
options.linAlg = 'adap';
% options.timeStep = 0.05; 
% options.taylorTerms = 10; 
% options.zonotopeOrder = 5;

% Reachability Analysis --------------------------------------------------

[R1, res1] = reach(obj1, params, options, spec);
[R2, res2] = reach(obj2, params, options, spec);

%% Simulation -----------------------------------------------------------

%simulation options
simOpt.points = 20;
simOpt.vertSamp = 0;
simOpt.strechFac = 1.5;

% simulate using Rapidly Exploring Random Trees
simRes1 = simulateRRT(obj1, R1, params, simOpt);
simRes2 = simulateRRT(obj2, R2, params, simOpt);

%% Plot -------------------------------------------------------------------

figure(3)
plotOverTime(R1, 1, 'r');
hold on;
plotOverTime(R2,1, 'b');
legend('Actual', 'Estimated');
legend;
title('Reachabilty of Actual and Predicted System Using Cora');
xlabel('Time');
ylabel('Amplitude');
grid on;
saveas(gcf, 'Reach_cora.png');

figure(4)
plotOverTime(simRes1)
hold on;
plotOverTime(simRes2)
legend('Actual', 'Predicted');
title('Simulated Reachability of Actual and Predicted System Using Cora');
xlabel('Time');
ylabel('Amplitude');
grid on;
saveas(gcf, 'Simulation_cora.png');

figure(5)
plotOverTime(R2,1, 'g')
hold on;
plotOverTime(simRes2, 1,'*r')
legend('Reachable States', 'Simulated Path');
title('Simulated Path and Reachable States for Predicted System Using Cora');
xlabel('Time');
ylabel('Amplitude');
grid on;
saveas(gcf, 'Sim_Reach_cora.png');

%% Reachability -- NNV

sys = LinearODE(sys_ss.A, sys_ss.B, sys_ss.C, sys_ss.D);
sys2 = LinearODE(A1, B1, C1, D1);

% simulation params
h = 0.25; 
N = 20;
t = [0:h:N]; % Array of time samples, .25 increment up to N
u = zeros(1, length(t)); % input array
x0 = [2;2.5]; % first element is starting amp, the second is starting u
[y, t, x] = simulate(sys, u, t, x0);
[y2, t2, x2] = simulate(sys2, u, t, x0);

close all;
figure(1)
plot(t, y, 'r');
hold on;
plot(t2, y2, '--b');
title('Simulation of Actual and Predicted Using NNV');
ylabel('Amplitude');
xlabel('Time');
grid on;
legend('Actual', 'Predicted');
saveas(gcf, 'sim_nnv.png');


%% LINEAR ODE simREACH
method = 'direct';
h = 0.1; % time-step
N = 400; % number of steps
U = []; % control input set
R0 = Star([0; 2],[0.5; 2.25]);% initial set of states
R = simReach(sys, method, R0, U, h, N);
R2 = simReach(sys2, method, R0, U, h, N);

%% PLOT
close all;
Star.plots(R);
hold on;
Star.plots(R2, 'r');
title('Reachability of Actual and Predicted Using NNV');
xlabel('x1')
ylabel('x2');
grid on;
legend('Actual', 'Predicted');
saveas(gcf, 'Sim_Reach_nnv.png');