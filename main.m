warning off;
close all;
clear all;
clc;


rng('default');
%% rand pont generation
P = rand(2, 50);

%% Set point

X = rand(2, 50);

%% nearest neighbour

I = nearestneighbour(P, X);


%% Plot the points

figure(1)
plot(X(1,:), X(2,:), 'b.', P(1,:), P(2,:), 'r.', 'MarkerSize', 15);
hold on
quiver(P(1, :), P(2, :), X(1,I) - P(1, :), X(2, I) - P(2, :), 0, 'k');
hold off

%% Nearest neighbour to subset of a point set

X = rand(4, 20);

%% Find nearest neighbour  features

I = nearestneighbour([2 4 10], X);

%%  Find nearest neighbours to all columns 

I = nearestneighbour(X);

%% Example 3: Finding nearest n neighbours

P = rand(2, 3);

%% Candidate feature set

X = rand(2, 50);

%% Calculate the 4 nearest neighbours to each point: I(:, 1) will be the 4


I = nearestneighbour(P, X, 'NumberOfNeighbours', 4);

%%  Plot the points - show the neighbours for one of them

figure(2)
plot(P(1,:), P(2, :), 'r.', X(1,:), X(2,:), 'b.', 'MarkerSize', 15)
hold on
p1 = repmat(P(1,1), 1, 4); p2 = repmat(P(2,1), 1, 4);
quiver(p1, p2, X(1, I(:, 1)) - p1, X(2, I(:, 1)) - p2, 0, 'k')
hold off

%% Example 4: Finding neighbours within a specified radius

%% three points of interest

P = rand(2, 3);

%%  Candidate feature set

X = rand(2, 50);

I = nearestneighbour(P, X, 'Radius', 0.2);

%% Plot the points - show the neighbours for one of them

idx = I(I(:, 1) ~= 0, 1);
plot(P(1,:), P(2, :), 'r.', X(1,:), X(2,:), 'b.', 'MarkerSize', 15)
hold on
p1 = repmat(P(1,1), 1, length(idx)); p2 = repmat(P(2,1), 1, length(idx));
quiver(p1, p2, X(1, idx) - p1, X(2, idx) - p2, 0, 'k')
hold off

%% Combining NumberOfNeighbours and Radius

%% Point of interest
P = rand(2, 1);

% Candidate feature set
X = rand(2, 30);

%% Calculate the  nearest neighbours Euclidean distance

I = nearestneighbour(P, X, 'n', 5, 'r', 0.2);
n = length(I);

%% Plot the points - show the neighbours for one of them
figure(3)
plot(P(1,:), P(2, :), 'r.', X(1,:), X(2,:), 'b.', 'MarkerSize', 15)
hold on

quiver(repmat(P(1), 1, n), repmat(P(2), 1, n), X(1, I) - P(1), X(2, I) - P(2), 0, 'k')
hold off


%% Large Data set, few points to look up
P = rand(3, 10);
X = rand(3, 5000);

