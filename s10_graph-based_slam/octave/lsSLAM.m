more off;
clear all;
close all;

addpath('tools');

% load the graph into the variable g
% only leave one line uncommented

% simulation datasets
load ../data/simulation-pose-pose.dat
%load ../data/simulation-pose-landmark.dat

% real-world datasets
%load ../data/intel.dat
%load ../data/dlr.dat

% plot the initial state of the graph
plot_graph(g, 0);

printf('Initial error %f\n', compute_global_error(g));

% the number of iterations
numIterations = 100;

% maximum allowed dx
EPSILON = 10^-4;

% Error
err = 0;

% carry out the iterations
for i = 1:numIterations
  printf('Performing iteration %d\n', i);

  dx = linearize_and_solve(g);

  % TODO: apply the solution to the state vector g.x

  % plot the current state of the graph
  plot_graph(g, i);

  err = compute_global_error(g);

  % Print current error
  printf('Current error %f\n', err);

  % TODO: implement termination criterion as suggested on the sheet

end

printf('Final error %f\n', err);
