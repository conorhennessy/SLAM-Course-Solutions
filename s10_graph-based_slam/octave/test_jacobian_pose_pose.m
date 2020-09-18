clear all;
close all;
more off;
addpath('tools');

epsilon = 1e-5;

x1 = [1.1 0.9 1]';
x2 = [2.2 1.85 1.2]';
z  = [0.9 1.1 1.05]';

% get the analytic Jacobian
[e, A, B] = linearize_pose_pose_constraint(x1, x2, z);

% check the error vector
e_true = [-1.06617  -1.18076  -0.85000]';
if (norm(e - e_true) > epsilon)
  disp('Your error function seems to return a wrong value');
  disp('Result of your function'); disp(e);
  disp('True value'); disp(e_true);
else
  disp('The computation of the error vector appears to be correct');
end

% compute it numerically
delta = 1e-6;
scalar = 1 / (2*delta);

% test for x1
ANumeric = zeros(3,3);
for d = 1:3
  curX = x1;
  curX(d) += delta;
  err = linearize_pose_pose_constraint(curX, x2, z);
  curX = x1;
  curX(d) -= delta;
  err -= linearize_pose_pose_constraint(curX, x2, z);

  ANumeric(:, d) = scalar * err;
end

diff = ANumeric - A;
if max(max(abs(diff))) > epsilon
  disp('Error in the Jacobian for x1');
  disp('Your analytic Jacobian'); disp(A);
  disp('Numerically computed Jacobian'); disp(ANumeric);
  disp('Difference'); disp(diff);
else
  disp('Jacobian for x1 appears to be correct');
end


% test for x2
BNumeric = zeros(3,3);
for d = 1:3
  curX = x2;
  curX(d) += delta;
  err = linearize_pose_pose_constraint(x1, curX, z);
  curX = x2;
  curX(d) -= delta;
  err -= linearize_pose_pose_constraint(x1, curX, z);

  BNumeric(:, d) = scalar * err;
end

diff = BNumeric - B;
if max(max(abs(diff))) > epsilon
  disp('Error in the Jacobian for x2');
  disp('Your analytic Jacobian'); disp(B);
  disp('Numerically computed Jacobian'); disp(BNumeric);
  disp('Difference'); disp(diff);
else
  disp('Jacobian for x2 appears to be correct');
end
