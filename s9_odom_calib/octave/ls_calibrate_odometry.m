% this function solves the odometry calibration problem
% given a measurement matrix Z.
% We assume that the information matrix is the identity
% for each of the measurements
% Every row of the matrix contains
% z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
% Z:	The measurement matrix
% X:	the calibration matrix
% returns the correction matrix X
function X = ls_calibrate_odometry(Z)
  % initial solution (the identity transformation)
  X = eye(3); 

  % TODO: initialize H and b of the linear system
  % Since H is linear combination of J'*Omega*J where
  % Init coefficient matrix, H
  H = zeros(9, 9);
  % Init coefficient vector, b
  b = zeros(9, 1);

  % Init info matrix, omega
  omega = eye(3, 3);

  %%% Note from @aimas-upb / slam-couse-solutions (https://github.com/aimas-upb/slam-course-solutions/blob/809de1a911226fa133c82538a54b4d76b5a956bb/odom_calib_framework/octave/ls_calibrate_odometry.m#L15)
  %% Since H is linear combination of J'*Omega*J where
  %% J is 9x3 and Omega is 3x3 it follows that H is 9x9
  %% Similarly, H is 9x9  and X 9x1, which results in b being 9x1.
  %% Omega is the information matrix of 3x3
  
  % TODO: loop through the measurements and update H and b
  % You may call the functions error_function and jacobian, see below
  % We assume that the information matrix is the identity.
  for i=1:size(Z, 1)
    % compute the jacobian and error for this point
    J = jacobian(i, Z);
    e = error_function(i, X, Z);
    
    % Update H
    H = H + J' * omega * J;
    % Update b
    b = b + J' * omega' * e;
  endfor
  % TODO: solve and update the solution
  delta_x = - inv(H) * b;  % Corresponds to first bullet point of slide 28, Least Sqaures lecture - 14
  X = X + [delta_x(1:3)'; delta_x(4:6)'; delta_x(7:9)'];
  %X += reshape( -inv(H)*b' , 3 , 3)';         % As @salihmarangoz/RobotMappingCourse said on line 29 of their solution
                                               % WHY TRANSPOSE WAS NEEDED? I COULDN'T FIND IT IN THE COURSE?
                                               % (https://github.com/salihmarangoz/RobotMappingCourse/blob/bd4f81eaa7730adf7fee83b19235212cd0319e29/7_odom_calibration/octave/ls_calibrate_odometry.m#L29)

end

% this function computes the error of the i^th measurement in Z
% given the calibration parameters
% i:	the number of the measurement
% X:	the actual calibration parameters
% Z:	the measurement matrix, each row contains first the scan-match result
%       and then the motion reported by odometry
% e:	the error of the ith measurement
function e = error_function(i, X, Z)
  % TODO compute the error of each measurement
  % Corresponds to second bullet point of slide 26, Least Sqaures lecture - 14
  % Note: I do not understand why both u values have to be transposed.
  u_truth = Z(i,1:3)';
  u_odom = Z(i,4:6)';
  e = u_truth - X * u_odom;
end

% derivative of the error function for the ith measurement in Z
% i:	the measurement number
% Z:	the measurement matrix
% J:	the jacobian of the ith measurement
function J = jacobian(i, Z)
  % TODO compute the Jacobian
  % Corresponds to last bullet point of slide 26, Least Sqaures lecture - 14
  J = zeros(3, 9);
  u_values = Z(i,4:6);
  J(1, 1:3) = u_values;
  J(2, 4:6) = u_values;
  J(3, 7:9) = u_values;
  J = -J;
end