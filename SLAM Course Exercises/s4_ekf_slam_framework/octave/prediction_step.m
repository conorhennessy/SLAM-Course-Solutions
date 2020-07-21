function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)

[m,n] = size(sigma);

v = zeros(m,1);

mu(1, 1) = mu(1) + (u.t * cos(mu(3) + u.r1));
mu(2, 1) = mu(2) + (u.t * sin(mu(3) + u.r1));
mu(3, 1) = mu(3) + (u.r1 + u.r2);
mu(3, 1) = normalize_angle(mu(3, 1));

% Compute the 3x3 Jacobian Gx of the motion model
Gx = eye(3, 3);
Gx(1, 3) = -u.t * cos(mu(3) + u.r1);
Gx(2, 3) = u.t * sin(mu(3) + u.r1);

% Construct the full Jacobian G
G = [Gx, zeros(3, n-3); zeros(n-3, 3), eye(n-3)];

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% Compute the predicted sigma after incorporating the motion
sigma = G*sigma*G' + R

end
