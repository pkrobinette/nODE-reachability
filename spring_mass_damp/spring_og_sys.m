%% System Identification for the Mass Spring Damper System
% TO DO:
% Assess these with bOxes of data -- a reach set.
% Figure out a way to do this

m = 10;     % mass
k = 0.8;    % spring constant
c = 0.5;    % damper constant
Ts = 0.05;  % sampling time
n = 1e-5;   % noise coefficient

A = [0 1; -k/m -c/m];
B = [0; 1/m];
C = [1 0];
D = [0];

% can either make state space or transfer function, does not matter
sys_ss = ss(A, B, C, D);
% act_sys = idtf(sys_ss);

% generating inputs to the system
[u,t] = gensig("square",500 , 1000, 1);
u = u+1;

% split into training and testing
N = 700;
train_u = u(1:N, 1);
train_t = t(1:N, 1);
test_u = u(N:end, 1);
test_t = t(N:end, 1);

[ay, at, ax] = lsim(sys_ss, train_u, train_t);

%% Add noise to the output data
noise = 0.15*randn(length(ay), 1);
ay_noise = ay + noise;
% figure(1);
% plot(at, ay, 'r');
% hold on
% plot(at, ay_noise, 'b');

%% sys id
s_data = iddata(ay_noise, train_u); % organize data
np = 2; % number of poles
nz = 0; % number of zeros
sys_test = tfest(s_data, np, nz); % estimate transfer function

% Obtain model parameters and associated uncertainty data
pvec = getpvec(sys_test);

% Compare data to estimated function
figure(1);
compare(s_data, sys_test)

[oy, ot, ox] = lsim(sys_ss, test_u, test_t);
[ey, et, ex] = lsim(sys_test, test_u, test_t);

act_data = iddata(oy, ox);
pred_data = iddata(ey, ex);

figure(2);
compare(act_data, pred_data);

%% Validate Model

% new inputs into the system
[u,t] = gensig("sine", 500 , 1000, 1);
u = u+1;

%% Simulate models

[act_y, act_t, act_x] = lsim(sys_ss, u, t);
[pred_y, pred_t, pred_x] = lsim(sys_test, u, t);

err = immse(act_y, pred_y);
disp(err)

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

%% Attempts at Reachabilty

% System Dynamics ---------------------------------------------------------

A1 = [0 1; -pvec(3) -pvec(2)];
B1 = [0; pvec(1)];
C1 = [1 0];
D1 = [0];

obj1 = linearSys(sys_ss.A, sys_ss.B,[], sys_ss.C, sys_ss.D);
obj2 = linearSys(A1, B1,[], C1, D1);

% Parameters ---------------------------------------------------------------

params.tFinal = 20;
params.R0 = zonotope(interval([2;2], [2.5;2.5]));
params.U = zonotope(interval(0, 0.5));

% Specifications ------------------------------

S = interval(10, 12);
spec = specification(S, 'unsafeSet');

% Reachability Settings ---------------------------------------------------
options.linAlg = 'adap';
options.timeStep = 0.05; 
% options.taylorTerms = 10; 
% options.zonotopeOrder = 5;

% Reachability Analysis --------------------------------------------------

[R1, res1] = reach(obj1, params, options, spec);
[R2, res2] = reach(obj2, params, options, spec);

% Simulation -----------------------------------------------------------

%simulation options
simOpt.points = 20;
simOpt.vertSamp = 0;
simOpt.strechFac = 1.5;

% simulate using Rapidly Exploring Random Trees
simRes1 = simulateRRT(obj1, R1, params, simOpt);
simRes2 = simulateRRT(obj2, R2, params, simOpt);

% Plot -------------------------------------------------------------------

figure(2)
plotOverTime(R1, 1, '--r');
hold on
plotOverTime(R2, 1, '--b');

figure(3)
plot(simRes1)
title("Rapidly Random Tree Simulation of Actual System");

figure(4)
plot(simRes2)
title("Rapidly Random Tree Simulation of Predicted System");

%% Simulation Wave --------------------------------------------------------------

% parameter
params_sim.x0 = [6;4]; 
params_sim.tFinal = 200;
params_sim.u = [0.1 0 -0.1 0.2];

% simulation
[t,x] = simulate(obj1 ,params_sim);
[t2, x2] = simulate(obj2, params_sim);

figure(3)
plot(t,x(:, 1), 'r', t2, x2(:,1), '--b')
title('Actual vs. Predicted Simulation')
xlabel('t')
ylabel('x')

%% Simulate RRT -----------------------------------------------
%simulation options
simOpt.points = 10;
simOpt.vertSamp = true;
simOpt.strechFac = 1;

% simulate using Rapidly Exploring Random Trees
simRes1 = simulateRRT(obj1, R1, params, simOpt);
simRes2 = simulateRRT(obj2, R2, params, simOpt);

%%
figure(4)
plot(simRes1);
plot(simRes2);

%% Visualization -----------------------------------------------------------

% plot different projections
dims = {[1,2],[3 4]};

for k = 1:length(dims)
    
    figure; hold on; box on
    projDims = dims{k};

    % plot reachable sets 
    plot(R,projDims,'FaceColor',[.8 .8 .8],'EdgeColor','none');
    
    % plot initial set
    plot(params.R0,projDims,'w-','lineWidth',2);
    
    % plot simulation results
    plot(simRes,projDims,'y');

    % label plot
    xlabel(['x_{',num2str(projDims(1)),'}']);
    ylabel(['x_{',num2str(projDims(2)),'}']);
end

% example completed
res = 1;

%------------- END OF CODE --------------

%% Test scenario

A = [-2 0; 1 -3];
B = [1; 1];
C = [1 0];

sys = linearSys(A,B,[],C);

% system dynamics
% sys = linearSys([-0.7 -2;2 -0.7],[1;1],0, [-2;-1], []);
% specs
S = interval(10,12);
spec = specification(S, 'unsafeSet');
% parameter
params.tFinal = 5;
params.R0 = zonotope(interval([2;2],[2.5;2.5])); 
params.U = zonotope(interval(-0.1,0.1));
% reachability settings
options.timeStep = 0.05; 
options.zonotopeOrder = 10; 
options.taylorTerms = 5;
% reachability analysis
[R, res] = reach(sys,params,options, spec);



