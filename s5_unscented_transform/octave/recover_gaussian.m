function [mu, sigma] = recover_gaussian(sigma_points, w_m, w_c)
% This function computes the recovered Gaussian distribution (mu and sigma)
% given the sigma points (size: nx2n+1) and their weights w_m and w_c:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n].
% The weight vectors are each 1x2n+1 in size,
% where n is the dimensionality of the distribution.

% Try to vectorize your operations as much as possible

n = size(sigma_points, 1);

% init mean (mu) and covariance (sigma)
mu = zeros(n, 1);
sigma = zeros(n, n);

% Compute the mean (mu) and covariance (sigma) of the resulting distribution given the transformed sigma points and corresponding weights.
% TODO: compute mu
for i = 1:2*n+1
    mu = mu + w_m(1, i) * sigma_points(:, i);
endfor

% TODO: compute sigma
for i = 1:2*n+1
    sigma = sigma + w_c(1, i) * (sigma_points(:, i) - mu) * (sigma_points(:, i) - mu)';
endfor

end
