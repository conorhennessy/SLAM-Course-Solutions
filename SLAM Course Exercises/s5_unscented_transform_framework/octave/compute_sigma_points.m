function [sigma_points, w_m, w_c] = compute_sigma_points(mu, sigma, lambda, alpha, beta)
% This function samples 2n+1 sigma points from the distribution given by mu and sigma
% according to the unscented transform, where n is the dimensionality of mu.
% Each column of sigma_points should represent one sigma point
% i.e. sigma_points has a dimensionality of nx2n+1.
% The corresponding weights w_m and w_c of the points are computed using lambda, alpha, and beta:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n] (i.e. each of size 1x2n+1)
% They are later used to recover the mean and covariance respectively.

n = length(mu);
sigma_points = zeros(n, 2*n+1);

%init the weights
w_m = zeros(1, 2*n+1);
w_c = zeros(1, 2*n+1);

% TODO: compute all sigma points
% First sigma point is the mean
sigma_points(:, 1) = mu;
% For remaining sigma points
% first, compute alternative to the square root matrix, the Cholesky decompositon by Cholesky matrix (sigma = L * L^T)
cholesky_matrix = chol((n+lambda)*sigma, "lower");
for i = 1:2n
    if i <= n
        sigma_points(:, i+1) = mu + cholesky_matrix(:, i);
    endif
    if i > n
        sigma_points(:, i+1) = mu - cholesky_matrix(:, i);
    endif
endfor

% TODO compute weight vectors w_m mean and w_c covariance
% Weights for first sigma value
w_m(1,1) = lambda / (n + lambda);
w_c(1, 1) = w_m(1, 1) + (1 - power(alpha, 2) + beta);
% Weights for remaining sigma values
w_m(1, 2:end) = 1 / (2 * (n + lambda));
w_c(1, 2:end) = w_m(1, 2:end);

end
