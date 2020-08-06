function [mu, sigma, sigma_points] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model.
% mu: state vector containing robot pose and poses of landmarks obeserved so far
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% For computing lambda.
global scale;

% Compute sigma points
sigma_points = compute_sigma_points(mu, sigma);

% Dimensionality
n = length(mu);
% lambda
lambda = scale - n;

% TODO: Transform all sigma points according to the odometry command
% Remember to vectorize your operations and normalize angles
% Tip: the function normalize_angle also works on a vector (row) of angles
%sigma_points(1, :) = sigma_points(1, :) + u.t * normalize_angle(cos(sigma_points(1, :) + u.r1));
%sigma_points(2, :) = sigma_points(2, :) + u.t * normalize_angle(sin(sigma_points(2, :) + u.r1)); 
%sigma_points(3, :) = normalize_angle(sigma_points(3, :) + u.r1 + u.r2);
for i=1:2*n+1
    sigma_points(1:3,i) = sigma_points(1:3,i) + [u.t*cos(sigma_points(3,i) + u.r1); u.t*sin(sigma_points(3,i) + u.r1); u.r1 + u.r2];
    sigma_points(3,i) = normalize_angle(sigma_points(3,i));
endfor

% Computing the weights for recovering the mean
wm = [lambda/scale, repmat(1/(2*scale),1,2*n)];
wc = wm;

% TODO: recover mu.
% Be careful when computing the robot's orientation (sum up the sines and
% cosines and recover the 'average' angle via atan2)
mu = zeros(n, 1);
x_bar = 0;
y_bar = 0;
for i = 1:2*n+1
    mu = mu + wm(1, i) * sigma_points(:, i);
    x_bar = x_bar + wm(i) * cos(sigma_points(3,i));
    y_bar = y_bar + wm(i) * sin(sigma_points(3,i));
endfor
mu(3) = atan2(y_bar, x_bar);
mu(3) = normalize_angle(mu(3));

% TODO: Recover sigma. Again, normalize the angular difference
sigma = zeros(n, 1);
for i = 1:2*n+1
     diff = normalize_angle(sigma_points(:, i) - mu);
     sigma = sigma + wc(1, i) * diff * diff';
endfor

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Add motion noise to sigma
sigma = sigma + R;

end
