% This is the main unscented Kalman filter SLAM loop. This script calls all the required
% functions in the correct order.
%
% You can disable the plotting or change the number of steps the filter
% runs for to ease the debugging. You should however not change the order
% or calls of any of the other lines, as it might break the framework.
%
% If you are unsure about the input and return values of functions you
% should read their documentation which tells you the expected dimensions.

% Turn off pagination:
close all
clear all
more off;

format long

% Make tools available
addpath('tools');

% Read world data, i.e. landmarks. The true landmark positions are not given to the robot
landmarks = read_world('../data/world.dat');
%load landmarks
% Read sensor readings, i.e. odometry and range-bearing sensor
data = read_data('../data/sensor_data.dat');
%load data
% Initialize belief
mu = zeros(3,1);
sigma = 0.001*eye(3);
map = [];

% For computing lambda
% scale = lambda + dimensionality
global scale;
scale = 3.0;

% toogle the visualization type
showGui = true;  % show a window while the algorithm runs
%showGui = false; % plot to files instead

% Perform filter update for each odometry-observation pair read from the
% data file.
for t = 1:size(data.timestep, 2)
%for t = 1:25
    disp('Time step t ='), disp(t)

    % Perform the prediction step of the UKF
    [mu, sigma] = prediction_step(mu, sigma, data.timestep(t).odometry);

    % Perform the correction step of the UKF
    [mu, sigma, map] = correction_step(mu, sigma, data.timestep(t).sensor, map);

    %Generate visualization plots of the current state of the filter
    plot_state(mu, sigma, landmarks, t, map, data.timestep(t).sensor, showGui);
    disp("Current state vector mu ="), disp(mu)
    disp("Map contains the following landmarks:"), disp(map)

endfor

disp("Final system covariance matrix:"), disp(sigma)

% Display the final state estimate
disp("Final robot pose:")
disp("mu_robot = "), disp(mu(1:3)), disp("sigma_robot = "), disp(sigma(1:3,1:3))
