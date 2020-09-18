% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_constraint(x, l, z)

  % TODO compute the error and the Jacobians of the error
  % Homogenous transformation of the pose 
  X = v2t(x);

  % Pose rotation matrix
  Ri = X(1:2, 1:2);

  % Erorr for landmark observation
  % Eij = R' (Xj -Ti) - Zij
  e = Ri' * (l - x(1:2)) - z;       % This corresponds to second line of slide 10 of lecture 17

  % Compute pose relative angle. 
  % Note: that using atan2 also normilizes the angle.
  theta_i = atan2(Ri(2,1), Ri(1,1));
  
  % Set the robot pose coordinates
  xi = x(1); 
  yi = x(2);
  
  % Set the landmark coordinates
  xl = l(1); 
  yl = l(2);

  % Compute error function eij partial derivative with respect to pose x
  eij_xi = [-cos(theta_i); sin(theta_i)];
  eij_yi = [-sin(theta_i); -cos(theta_i)];
  eij_theta_i = [-(xl-xi)*sin(theta_i)+(yl-yi)*cos(theta_i);
                 -(xl-xi)*cos(theta_i)-(yl-yi)*sin(theta_i)
                ];
                
  % Construct Jacobian block A with respect to x
  A = [eij_xi, eij_yi, eij_theta_i];
  
  % Compute eij partial derivative with respect to pose x
  eij_xl = [cos(theta_i); -sin(theta_i)];
  eij_yl = [sin(theta_i); cos(theta_i)];
  
  % Construct Jacobian block B with respect to landmark l
  B = [eij_xl, eij_yl];

end;
