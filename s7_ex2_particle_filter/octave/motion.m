% Turn off pagination:
more off;

clear all;
close all;

% Make tools available
addpath('tools');

% Read sensor readings, i.e. odometry
data = read_data('../data/odometry.dat');

noise = [0.005, 0.01, 0.005]';

% how many particles
numParticles = 100;

% initialize the particles array
particles = struct;
for i = 1:numParticles
  particles(i).weight = 1. / numParticles;
  particles(i).pose = zeros(3,1);
  particles(i).history = cell();
end

% Perform filter update for each odometry-observation read from the
% data file.
for t = 1:size(data.timestep, 2)
%for t = 1:50
    printf('timestep = %d\n', t);

    % Perform the prediction step of the particle filter
    particles = prediction_step(particles, data.timestep(t).odometry, noise);

    % Generate visualization plots of the current state of the filter
    plot_state(particles, t);
end
