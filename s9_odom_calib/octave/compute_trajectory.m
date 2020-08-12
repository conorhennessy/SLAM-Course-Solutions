% computes the trajectory of the robot by chaining up
% the incremental movements of the odometry vector
% U:	a Nx3 matrix, each row contains the odoemtry ux, uy utheta
% T:	a (N+1)x3 matrix, each row contains the robot position (starting from 0,0,0)
function T = compute_trajectory(U)
  % initialize the trajectory matrix
  T = zeros(size(U,1)+1, 3);
  % store the first pose in the result
  T(1, :) = zeros(1,3);
  % the current pose in the chain
  currentPose = v2t(T(1, :));

  % TODO: compute the result of chaining up the odometry deltas
  % Note that U(i) results in T(i+1).
  % T(i+1) can be computed by calling t2v(currentPose)
  % after computing the current pose of the robot
  for i = 1:size(U)
    % compute the current pose of the robot at i
    currentPose = currentPose * v2t(T(i, :));
    % T(i+1) can be computed by calling t2v(currentPose)
    % Thus add this value to the T matrix
    T(i + 1, :) = t2v(currentPose);
  endfor
end
