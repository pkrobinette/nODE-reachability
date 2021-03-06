% Just practicing neural ode's in matlab
% from https://www.mathworks.com/help/deeplearning/ug/train-neural-ode-network.html
%% calculate x,y vars from an equation. Just like t, vars in buck converter

x0 = [2; 0];
A = [-0.1 -1; 1 -0.1];

numTimeSteps = 4000;
T = 15;
odeOptions = odeset('RelTol', 1.e-7, 'AbsTol', 1.e-9);
t = linspace(0, T, numTimeSteps);
[~, x] = ode45(@(t,y) A*y, t, x0, odeOptions);
x = x';

%% 

hiddenSize = 30;
inputSize = size(x,1);
outputSize = inputSize;

neuralOdeLayers = [
    fullyConnectedLayer(hiddenSize)
    tanhLayer
    fullyConnectedLayer(hiddenSize)
    tanhLayer
    fullyConnectedLayer(outputSize)
    ];

neuralOdeInternalDlnetwork = dlnetwork(neuralOdeLayers,'Initialize',false);
neuralOdeInternalDlnetwork.Learnables

neuralOdeInternalTimesteps = 40;
dt = t(2);
neuralOdeLayerName = 'neuralOde';

customNeuralOdeLayer = neuralOdeLayer(neuralOdeInternalDlnetwork,neuralOdeInternalTimesteps,dt,neuralOdeLayerName);

dlnet=dlnetwork(customNeuralOdeLayer,'Initialize',false);
dlnet = initialize(dlnet, dlarray(ones(inputSize,1),'CB'));

gradDecay = 0.9;
sqGradDecay = 0.999;
learnRate = 0.001;

numIter = 1500;
miniBatchSize = 200;

plots = "training-progress";
lossHistory = [];

plotFrequency = 50;

averageGrad = [];
averageSqGrad= [];

if plots == "training-progress"
    figure(1)
    clf
    title('Training Loss');
    lossline = animatedline;
    xlabel('Iteration')
    ylabel("Loss")
    grid on
end
numTrainingTimesteps = numTimeSteps;
trainingTimesteps = 1:numTrainingTimesteps;

start = tic;

for iter=1:numIter
    
    % Create batch 
    [dlx0, targets] = createMiniBatch(numTrainingTimesteps, neuralOdeInternalTimesteps, miniBatchSize, x);
    
    % Evaluate network and compute gradients 
    [grads,loss] = dlfeval(@modelGradients,dlnet,dlx0,targets);
    
    % Update network 
    [dlnet,averageGrad,averageSqGrad] = adamupdate(dlnet,grads,averageGrad,averageSqGrad,iter,...
        learnRate,gradDecay,sqGradDecay);
    
    % Plot loss
    currentLoss = extractdata(loss);
    
    if plots == "training-progress"
        addpoints(lossline, iter, currentLoss);
        drawnow
    end
    
    % Plot predicted vs. real dynamics
    if mod(iter,plotFrequency) == 0
        figure(2)
        clf

        % Extract the learnt dynamics
        internalNeuralOdeLayer = dlnet.Layers(1);
        dlnetODEFcn = @(t,y) evaluateODE(internalNeuralOdeLayer, y);

        % Use ode45 to compute the solution 
        [~,y] = ode45(dlnetODEFcn, [t(1) t(end)], x0, odeOptions);
        y = y';
        
        plot(x(1,trainingTimesteps),x(2,trainingTimesteps),'r--')
        hold on
        plot(y(1,:),y(2,:),'b-')
        hold off
        D = duration(0,0,toc(start),'Format','hh:mm:ss');
        title("Iter = " + iter + ", loss = " + num2str(currentLoss) + ", Elapsed: " + string(D))
        legend('Training ground truth', 'Predicted')
    end
end

function [dlX0, dlT] = createMiniBatch(numTimesteps, numTimesPerObs, miniBatchSize, X)
% Create batches of trajectories
s = randperm(numTimesteps - numTimesPerObs, miniBatchSize);

dlX0 = dlarray(X(:, s),'CB');
dlT = zeros([size(dlX0,1) miniBatchSize numTimesPerObs]);

for i = 1:miniBatchSize
    dlT(:, i, 1:numTimesPerObs) = X(:, s(i):(s(i) + numTimesPerObs - 1));
end
end

function [gradients,loss] = modelGradients(dlnet, dlX0, targets)

% Compute prediction of network
dlX = forward(dlnet,dlX0);

% Compute mean absolute error loss
loss = sum(abs(dlX - targets), 'all') / numel(dlX);

% Compute gradients
gradients = dlgradient(loss,dlnet.Learnables);

end

function [xPred, xTrue, error] = predictWithOde45(dlnet,A,tPred,x0Pred,odeOptions)
% Use ode45 to compute the solution both with the true and the learnt
% models.

[~, xTrue] = ode45(@(t,y) A*y, tPred, x0Pred, odeOptions);

% Extract the learnt dynamics
internalNeuralOdeLayer = dlnet.Layers(1);
dlnetODEFcn = @(t,y) evaluateODE(internalNeuralOdeLayer, y);

[~,xPred] = ode45(dlnetODEFcn, tPred, x0Pred, odeOptions);
error = mean(abs(xTrue - xPred), 'all');
end

function plotTrueAndPredictedSolutions(xTrue, xPred, err, x0Str)
plot(xTrue(:,1),xTrue(:,2),'r--',xPred(:,1),xPred(:,2),'b-','LineWidth',1)
title("x_0 = " + x0Str + ", err = " + num2str(err) )
xlabel('x1')
ylabel('x2')
xlim([-2 2])
ylim([-2 2])
legend('Ground truth', 'Predicted')
end