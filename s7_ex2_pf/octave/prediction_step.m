function particles = prediction_step(particles, u, noise)
% Updates the particles by drawing from the motion model
% Use u.r1, u.t, and u.r2 to access the rotation and translation values
% which have to be pertubated with Gaussian noise.
% The position of the i-th particle is given by the 3D vector
% particles(i).pose which represents (x, y, theta).

% noise parameters
% Assume Gaussian noise in each of the three parameters of the motion model.
% These three parameters may be used as standard deviations for sampling.
r1Noise = noise(1);
transNoise = noise(2);
r2Noise = noise(3);

numParticles = length(particles);

for i = 1:numParticles
  % append the old position to the history of the particle
  particles(i).history{end+1} = particles(i).pose;

  % TODO: sample a new pose for the particle

  % First update the robot particles according to the motion model
  % By computing the new mu based on the noise-free (odometry-based) motion model
  %mu(1) = particles(i).pose + (u.t * cos(particles(i).pose(3) + u.r1));
  %mu(2) = particles(i).pose + (u.t * sin(particles(i).pose(3) + u.r1));
  %mu(3) = particles(i).pose + (u.r1 + u.r2);
  mu = particles(i).pose + [u.t * cos(particles(i).pose(3) + u.r1); u.t * sin(particles(i).pose(3) + u.r1); u.r1 + u.r2];
  mu(3) = normalize_angle(mu(3));

  % Then, update particles(i) according to the motion represented by u and the noise
  %%  Note: The function normrnd(µ, σ) allows to draw samples from a Gaussian with
  %%  mean µ and standard deviation σ. Where µ is mu & σ is standard deviation
  %%  Standard deviation is given by the sum of noise parameters as required.
  particles(i).pose(1) = normrnd(mu(1), r1Noise + transNoise + r2Noise);
  particles(i).pose(2) = normrnd(mu(2), r1Noise + transNoise + r2Noise);
  particles(i).pose(3) = normrnd(mu(3), r1Noise + r2Noise);

end

end