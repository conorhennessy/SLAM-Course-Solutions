% This is the main extended Kalman filter SLAM loop. This script calls all the required
% functions in the correct order.
%
% You can disable the plotting or change the number of steps the filter
% runs for to ease the debugging. You should however not change the order
% or calls of any of the other lines, as it might break the framework.
%
% If you are unsure about the input and return values of functions you
% should read their documentation which tells you the expected dimensions.

% Turn off pagination:
more off;

% clear all variables and close all windows
clear all;
close all;

% Make tools available
%addpath('tools');

% Read world data, i.e. landmarks. The true landmark positions are not given to the robot
landmarks = read_world('world.dat');
% load landmarks;
% Read sensor readings, i.e. odometry and range-bearing sensor
data = read_data('sensor_data.dat');
%load data;

INF = 1000;
% Get the number of landmarks in the map
N = size(landmarks,2);

% observedLandmarks is a vector that keeps track of which landmarks have been observed so far.
% observedLandmarks(i) will be true if the landmark with id = i has been observed at some point by the robot
observedLandmarks = repmat(false,1,N);

% Initialize belief:
% mu: 2N+3x1 vector representing the mean of the normal distribution
% The first 3 components of mu correspond to the pose of the robot,
% and the landmark poses (xi, yi) are stacked in ascending id order.
% sigma: (2N+3)x(2N+3) covariance matrix of the normal distribution
mu = repmat([0.0], (2*N+3), 1);
robSigma = zeros(3);
robMapSigma = zeros(3,2*N);
mapSigma = INF*eye(2*N);
sigma = [[robSigma robMapSigma];[robMapSigma' mapSigma]];

% toogle the visualization type
%showGui = true;  % show a window while the algorithm runs
showGui = false; % plot to files instead

% Perform filter update for each odometry-observation pair read from the
% data file.
for t = 1:size(data.timestep, 2)
%for t = 1:80

    % Perform the prediction step of the EKF
    [mu, sigma] = prediction_step(mu, sigma, data.timestep(t).odometry);

    % Perform the correction step of the EKF
    [mu, sigma, observedLandmarks] = correction_step(mu, sigma, data.timestep(t).sensor, observedLandmarks);

    %Generate visualization plots of the current state of the filter
    plot_state(mu, sigma, landmarks, t, observedLandmarks, data.timestep(t).sensor, showGui);
    disp("Current state vector:")
    disp("mu = "), disp(mu)
endfor

disp("Final system covariance matrix:"), disp(sigma)
% Display the final state estimate
disp("Final robot pose:")
disp("mu_robot = "), disp(mu(1:3)), disp("sigma_robot = "), disp(sigma(1:3,1:3))
