% This is the main FastSLAM loop. This script calls all the required
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

clear all;
close all;

% Make tools available
addpath('tools');

% Read world data, i.e. landmarks. The true landmark positions are not given to the robot
landmarks = read_world('../data/world.dat');
% Read sensor readings, i.e. odometry and range-bearing sensor
data = read_data('../data/sensor_data.dat');

% Get the number of landmarks in the map
N = size(landmarks,2);

noise = [0.005, 0.01, 0.005]';

% how many particles
numParticles = 100;

% initialize the particles array
particles = struct;
for i = 1:numParticles
  particles(i).weight = 1. / numParticles;
  particles(i).pose = zeros(3,1);
  particles(i).history = cell();
  for l = 1:N % initialize the landmarks aka the map
    particles(i).landmarks(l).observed = false;
    particles(i).landmarks(l).mu = zeros(2,1);    % 2D position of the landmark
    particles(i).landmarks(l).sigma = zeros(2,2); % covariance of the landmark
  end
end

% toogle the visualization type
%showGui = true;  % show a window while the algorithm runs
showGui = false; % plot to files instead

% Perform filter update for each odometry-observation pair read from the
% data file.
for t = 1:size(data.timestep, 2)
%for t = 1:50
    printf('timestep = %d\n', t);

    % Perform the prediction step of the particle filter
    particles = prediction_step(particles, data.timestep(t).odometry, noise);

    % Perform the correction step of the particle filter
    particles = correction_step(particles, data.timestep(t).sensor);

    % Generate visualization plots of the current state of the filter
    plot_state(particles, landmarks, t, data.timestep(t).sensor, showGui);

    % Resample the particle set
    particles = resample(particles);
end
