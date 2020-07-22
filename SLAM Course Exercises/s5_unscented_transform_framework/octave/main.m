% This is the main script for computing a transformed distribution according 
% to the unscented transform. This script calls all the required
% functions in the correct order.
% If you are unsure about the input and return values of functions you
% should read their documentation which tells you the expected dimensions.

% Turn off pagination and open a new figure for plotting
more off
close all
figure
hold on
grid

% Make tools available
addpath('tools');

% Initial distribution
sigma = 0.1*eye(2);
mu = [1;2];
n = length(mu);

% Compute lambda
alpha = 0.9;
beta = 2;
kappa = 1;
lambda = alpha*alpha*(n+kappa)-n;

% Compute the sigma points corresponding to mu and sigma
[sigma_points, w_m, w_c] = compute_sigma_points(mu, sigma, lambda, alpha, beta);

% Plot original distribution with sampled sigma points
plot(mu(1),mu(2),'ro','markersize',12, 'linewidth',3)
legend('original distribution')
drawprobellipse(mu, sigma, 0.9, 'r');
plot(sigma_points(1,:),sigma_points(2,:),'kx','markersize', 10, 'linewidth',3)

% Transform sigma points
sigma_points_trans = transform(sigma_points);

% Recover mu and sigma of the transformed distribution
[mu_trans, sigma_trans] = recover_gaussian(sigma_points_trans, w_m, w_m);

% Plot transformed sigma points with corresponding mu and sigma
plot(mu_trans(1),mu_trans(2),'bo','markersize', 12, 'linewidth',3)
legend('transformed distribution')
drawprobellipse(mu_trans, sigma_trans, 0.9, 'b');
plot(sigma_points_trans(1,:),sigma_points_trans(2,:),'kx','markersize', 10, 'linewidth',3)

% Figure axes setup
title('Unscented Transform', 'fontsize', 20)
x_min = min(mu(1),mu_trans(1));
x_max = max(mu(1),mu_trans(1));
y_min = min(mu(2),mu_trans(2));
y_max = max(mu(2),mu_trans(2));
axis([(x_min-3) (x_max+3) (y_min-3) (y_max+3)], "equal")

% Print and save plot
print('../plots/unscented.png', '-dpng')
