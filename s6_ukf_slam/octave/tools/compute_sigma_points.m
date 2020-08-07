function [sigma_points] = compute_sigma_points(mu, sigma)
% Computes the 2n+1 sigma points according to the unscented transform,
% where n is the dimensionality of the mean vector mu.
% The sigma points should form the columns of sigma_points,
% i.e. sigma_points is an nx2n+1 matrix.

global scale;

% Compute lambda
n = length(mu);
num_sig = 2*n+1;
lambda = scale - n;

% Compute sigma points
sigmasqr = sqrtm(sigma);
sigmasqr = sqrt(n+lambda) * sigmasqr;

mureplicated = repmat(mu, 1, n);
sigma_points = [mu, mureplicated + sigmasqr, mureplicated - sigmasqr];

end
