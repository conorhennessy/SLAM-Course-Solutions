function particles = correction_step(particles, z)

% Weight the particles according to the current map of the particle
% and the landmark observations z.
% z: struct array containing the landmark observations.
% Each observation z(j) has an id z(j).id, a range z(j).range, and a bearing z(j).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.

% Number of particles
numParticles = length(particles);

% Number of measurements in this time step
m = size(z, 2);

% TODO: Construct the sensor noise matrix Q_t (2 x 2)
Q_t = [0.1, 0.0;
       0.0, 0.1]; 

% process each particle
for i = 1:numParticles
  robot = particles(i).pose;
  % process each measurement
  for j = 1:m
    % Get the id of the landmark corresponding to the j-th observation
    % particles(i).landmarks(l) is the EKF for this landmark
    l = z(j).id;

    % The (2x2) EKF of the landmark is given by
    % its mean particles(i).landmarks(l).mu
    % and by its covariance particles(i).landmarks(l).sigma

    % If the landmark is observed for the first time:
    if (particles(i).landmarks(l).observed == false)

      % TODO: Initialize its position based on the measurement and the current robot pose:
      % See to line 37 of correction_step in s4_ekf_SLAM for how this was simarly done previously in EKF. 
      robot_x = robot(1);
      robot_y = robot(2);
      robot_r = robot(3);
      particles(i).landmarks(1).mu = [ robot_x + z(j).range * cos(normalize_angle(z(j).bearing + robot_r));
                                       robot_y + z(j).range * sin(normalize_angle(z(j).bearing + robot_r))];

      % get the Jacobian with respect to the landmark position
      [h, H] = measurement_model(particles(i), z(j));

      % TODO: initialize the EKF for this landmark
      % This corresponds to line 9 of slide 28 of FAST SLAM lecture (12)
      particles(i).landmarks(l).sigma = inv(H) * Q_t * inv(H)';

      % Indicate that this landmark has been observed
      particles(i).landmarks(l).observed = true;

    else

      % get the expected measurement
      [expectedZ, H] = measurement_model(particles(i), z(j));

      % TODO: compute the measurement covariance
      % This corresponds to line 14 of slide 30 of FAST SLAM lecture (12)
      Q = (H * particles(i).landmarks(l).sigma * H') + Q_t;

      % TODO: calculate the Kalman gain
      % This corresponds to line 15 of slide 30 of FAST SLAM lecture (12)
      K = particles(i).landmarks(l).sigma * H' * inv(Q_t);

      % TODO: compute the error between the z and expectedZ (remember to normalize the angle)'
      % error for range and bearing
      Z_diff = [z(j).range - expectedZ(1); normalize_angle(z(j).bearing - expectedZ(2))];

      % TODO: update the mean and covariance of the EKF for this landmark
      % mean, this corresponds to line 16 of slide 30 of FAST SLAM lecture (12)
      particles(i).landmarks(1).mu = particles(i).landmarks(1).mu + K * (Z_diff);
      % covariance, this corresponds to line 17 of slide 30 of FAST SLAM lecture (12)
      particles(i).landmarks(1).sigma = (eye(2) - K * H) * particles(i).landmarks(1).sigma;

      % TODO: compute the likelihood of this observation, multiply with the former weight
      %       to account for observing several features in one time step
      % Weight, this corresponds to line 13 of slide 44 of FAST SLAM lecture (12)
      particles(i).weight = 1/sqrt(det(2 * pi * Q)) * exp(-0.5 * (Z_diff)' * inv(Q) * Z_diff);

    end

  end % measurement loop
end % particle loop

end
