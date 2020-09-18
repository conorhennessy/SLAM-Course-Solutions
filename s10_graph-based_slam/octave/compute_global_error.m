% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    %%% Note from @aimas-upb / slam-couse-solutions (https://github.com/aimas-upb/slam-course-solutions/blob/809de1a911226fa133c82538a54b4d76b5a956bb/lsslam_framework/octave/compute_global_error.m#L22)
    % Compute homogenous transformation of the measurement.
    % Note: In this case the measurement comes either from odometry
    % or from a virtual measurement obtained via scan matching.
    % Both types of information contain the angle.
    Z = v2t(edge.measurement);
    %information matrix
    omega = edge.information;

    %Error function
    % Calculate the error between poses     %This corresponds to slide 11 of lecture 15
    %error = t2v(inv(Z)*(inv(x1)*x2));
    error = Z\(x1\x2);
    % Recover the error vector
    eij = t2v(error);
    % Calculate the coefficent vector
    ei = eij' * omega * eij;                %This corresponds to slide 21 of lecture 15

    Fx = Fx + ei;

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.

    %%% From @aimas-upb / slam-couse-solutions (https://github.com/aimas-upb/slam-course-solutions/blob/809de1a911226fa133c82538a54b4d76b5a956bb/lsslam_framework/octave/compute_global_error.m#L42)
    % Compute homogenous transformation of the roobot pose vector
    X = v2t(x);
    
    % Compute homogenous transformation of the measurement
    % Note: In case of a landmark only the (x,y) position 
    % is given. Hence we must complete it to with a zero
    % for the angle.
    Z = v2t([edge.measurement; 0]);
    
    % Compute the homogenous transformation of the landmark position
    % given by the current graph. As for the actual measurement the
    % vector must be completed with an zero angle.
    L = v2t([l; 0]);
    
    % the information matrix is given by edge.information
    omega = edge.information;
    
    % compute the error between poses and landmarks 
    %e_tmp = t2v(inv(Z)*(inv(X)*L));
    e_tmp = t2v(Z\(X\L)); 
    e_tmp = e_tmp(1:2);
    
    eij = e_tmp' * omega * e_tmp;

    Fx = Fx + eij;
    
  end

end
