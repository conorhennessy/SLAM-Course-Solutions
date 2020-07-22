function [sigma_points, w_m, w_c] = compute_sigma_points(mu, sigma, lambda, alpha, beta)
% This function samples 2n+1 sigma points from the distribution given by mu and sigma
% according to the unscented transform, where n is the dimensionality of mu.
% Each column of sigma_points should represent one sigma point
% i.e. sigma_points has a dimensionality of nx2n+1.
% The corresponding weights w_m and w_c of the points are computed using lambda, alpha, and beta:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n] (i.e. each of size 1x2n+1)
% They are later used to recover the mean and covariance respectively.

n = length(mu);

% TODO: compute all sigma points
sigma_points = zeros(n, 2*n+1);
% First sigma point is the mean
sigma_points(:, 1) = mu;
% For remaining sigma points
% first, compute the square root matrix
sqrtm_matrix = sqrtm((n+lambda)*sigma);
for i=1:n
    sigma_points(:,i+1) = mu + sqrtm_matrix(:,i);
endfor
for i=n+1:2*n
    sigma_points(:,i+1) = mu - sqrtm_matrix(:,i-n);
endfor

% TODO compute weight vectors w_m mean and w_c covariance
%init the weights
w_m = zeros(1, 2*n+1);
w_c = zeros(1, 2*n+1);
% Weights for first sigma value
w_m(1,1) = lambda / (n + lambda);
w_c(1, 1) = w_m(1, 1) + (1 - power(alpha, 2) + beta);
% Weights for remaining sigma values
w_m(1, 2:end) = 1 / (2 * (n + lambda));
w_c(1, 2:end) = w_m(1, 2:end);

end
