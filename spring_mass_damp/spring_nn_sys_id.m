%% System Identification for the spring mass damper system using neural networks

clear all; close all; clc;

%ma = F - cv - kx
dt = 0.01;
tfinal = 100;
t = 0:dt:tfinal;

% Generate training data
input = []; output = [];
for j = 1:100
    x0 = 15*(rand(1, 2)-0.5);
    [t, x] = ode45(@spring, t, x0);
    input = [input; x(1:end-1,:)];
    output = [output; x(2:end,:)];
    figure(1)
    plot(x(:,1),x(:,2)), hold on
    plot(x0(1), x0(2), 'ro')
    figure(2)
    plot(t, x(:,1)), hold on
    plot(x0(1), 'ro');
end
figure(1)
title('Neural Network Training')
xlabel('x1')
ylabel('x2')
grid on;
figure(2)
title('Neural Network Training')
xlabel('Time')
ylabel('Amplitude')
grid on

%% Make and train network
net = feedforwardnet(10);
net.layers{1}.transferFcn = 'purelin';
net = train(net, input.', output.');


%% testing
figure(3)
x0 = 20*(rand(2,1)-0.5);
x_start = x0;
[t, x] = ode45(@spring, t, x0);
plot(x(:,1), x(:,2)), hold on
plot(x0(1), x0(2), 'ro', 'Linewidth', [2])
grid on;

xnn(1,:) = x0;
for jj=2:length(t)
    y0=net(x0);
    xnn(jj,:)= y0.';
    x0=y0;
end

%add onto fig 3
plot(xnn(:,1), xnn(:,2), ':', 'Linewidth', [2])
legend('Actual', 'Initial Location', 'NN Predicted');
title('Actual and Predicted System Using NN Sys Id');
xlabel('x1')
ylabel('x2')
grid on;

figure(4)
plot(t, x(:,1));
hold on
plot(0, x_start(1), 'ro', 'Linewidth', [2]);
hold on
plot(t, xnn(:, 1), ':', 'Linewidth', [2]);
title('Actual and Predicted System Using NN Over Time');
xlabel('Time');
ylabel('Amplitude');
legend('Actual', 'Initial Value', 'Predicted');
grid on;

%% If not training
% close all; clear; clc;
% load neural-net.mat;

%% Parse net
nnvNet = FFNNS.parse(net); % parse the network trained from matlab

%% Simulate network Compared to Actual
x0 = 20*(rand(2,1)-0.5);
x_start = x0;
[t, x] = ode45(@spring, t, x0);
lb = x_start;
ub = lb + 0.5;
I = Star(lb, ub);% initial set of states

figure(5)
plot(x(:,1), x(:,2)), hold on
plot(x0(1), x0(2), 'ro', 'Linewidth', [2])
grid on;

x2nn(1,:) = x0;
Rnn(1,:) = I;
for jj=2:length(t)
    y0=net(x0);
    [S, tt] = nnvNet.reach(I, 'approx-star');
    Rnn(jj,:)= S;
    I = Star(y0-0.25, y0+0.25);
    x2nn(jj,:)= y0.';
    x0=y0;
end

plot(x2nn(:,1), x2nn(:,2), ':', 'Linewidth', [2])
legend('Actual', 'Initial Location', 'NN Predicted');
title('Actual and Predicted System Using NN Sys Id');
xlabel('x1')
ylabel('x2')
grid on;

figure(6)
plot(t, x(:,1));
hold on
plot(0, x_start(1), 'ro', 'Linewidth', [2]);
hold on
plot(t, x2nn(:, 1), ':', 'Linewidth', [2]);
title('Actual and Predicted System Using NN Over Time');
xlabel('Time');
ylabel('Amplitude');
legend('Actual', 'Initial Value', 'Predicted');
grid on;

figure(7)
Star.plots(Rnn(1:25:length(t), 1));
hold on
plot(x2nn(:,1), x2nn(:,2), '--r')
title('Reachability of NN using NNV');
xlabel('x1');
ylabel('x2');
grid on;

%% Reachability
% lb = x_start;
% ub = lb + 0.5;
% I = Star(lb, ub);% initial set of states
% 
% Rnn(1,:) = I;
% for jj=2:length(t)
%     [S, tt] = nnvNet.reach(I, 'approx-star');
%     Rnn(jj,:)= S;
%     I = S;
% end
% 
% %%
% figure(7)
% Star.plots(Rnn(1:25:length(t), 1));
% hold on
% plot(x2nn(:,1), x2nn(:,2), '--r')
% title('Reachability of NN using NNV');
% xlabel('x1');
% ylabel('x2');
% grid on;

%% Save Images
saveas(figure(1), 'training_data-x.png');
saveas(figure(2), 'training_data-t.png');
% saveas(figure(3), 'sim-x.png');
% saveas(figure(4), 'sim-t.png');
saveas(figure(5), 'sim-x.png');
saveas(figure(6), 'sim-t.png');
saveas(figure(7), 'nn-reach.png');

%% Verify
% Unsafe = Star([10;12], [10;12]);
% [safe, vt, counterEx] = verify(I,Unsafe, 'star');

% Function would have to be updated if m, k, or c changed
% m = 10
% c = 0.5
% k = 0.8
function dxdt = spring(t,x)
    dxdt_1 = x(2);
    dxdt_2 = (-0.5/10)*x(2) - (0.8/10)*x(1);
    dxdt = [dxdt_1; dxdt_2];
end
