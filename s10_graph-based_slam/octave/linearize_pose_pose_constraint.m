% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error

% Compute the homogenous transformations
  X1 = v2t(x1);
  X2 = v2t(x2);
  Z = v2t(z);
  
  % Compute the error for this constraint
  %e = t2v(inv(Z)*(inv(X1)*X2));
  e = t2v(Z\(X1\X2));
  
  
  % Note: The code below only requires the computations of relative
  % measurement and pose relative angles. However, this will make the
  % code look a little involved. Thus, in order to compute the Jacobians blocks
  % each of the necesarry components for the computation are set separately. 
  
  % Set the measurment rotation matrix
  Rij = Z(1:2, 1:2);
  
  % Set pose X1 rotation matrix
  Ri = X1(1:2, 1:2);
  
  % Compute the pose X1 relative angle
  theta_i = atan2(Ri(2,1),Ri(1, 1));
  
  % Compute the measurement relative angle
  theta_ij = atan2(Rij(2,1),Rij(1, 1));
   
  % Set pose X1 coordinates
  xi = x1(1);
  yi = x1(2);
  
  % Set pose X2 coordinates
  xj = x2(1);
  yj = x2(2);
  
  % Compute eij partial derivative with respect to pose x1
  eij_xi = [-cos(theta_i)*cos(theta_ij)+sin(theta_i)*sin(theta_ij);
            cos(theta_i)*sin(theta_ij)+sin(theta_i)*cos(theta_ij);
            0
           ];

           
  eij_yi = [-sin(theta_i)*cos(theta_ij)-cos(theta_i)*sin(theta_ij);
            sin(theta_i)*sin(theta_ij)-cos(theta_i)*cos(theta_ij);
            0
           ];
           
  eij_theta_i = [-(xj - xi)*(sin(theta_i)*cos(theta_ij)+cos(theta_i)*sin(theta_ij))+(yj - yi)*(cos(theta_i)*cos(theta_ij)-sin(theta_i)*sin(theta_ij));
                 (xj - xi)*(sin(theta_i)*sin(theta_ij) - cos(theta_i)*cos(theta_ij))-(yj - yi)*(cos(theta_i)*sin(theta_ij)+sin(theta_i)*cos(theta_ij));
                 -1
                ];
                
  % Construct Jacobian block A with respect to x1             
  A = [eij_xi, eij_yi, eij_theta_i];

  % Compute the partial derivative with respect to pose x2
  eij_xj = [cos(theta_i)*cos(theta_ij)-sin(theta_i)*sin(theta_ij);
            -cos(theta_i)*sin(theta_ij)-sin(theta_i)*cos(theta_ij);
            0
           ];        
  eij_yj = [sin(theta_i)*cos(theta_ij)+cos(theta_i)*sin(theta_ij);
            -sin(theta_i)*sin(theta_ij)+cos(theta_i)*cos(theta_ij);
            0
           ];
            
  eij_theta_j = [0; 0; 1];
    
  % Construct Jacobian block B with respect to x2
  B = [eij_xj, eij_yj, eij_theta_j];  
end;
