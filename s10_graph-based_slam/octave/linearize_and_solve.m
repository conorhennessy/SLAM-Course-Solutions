% performs one iteration of the Gauss-Newton algorithm
% each constraint is linearized and added to the Hessian

function dx = linearize_and_solve(g)

nnz = nnz_of_graph(g);

% allocate the sparse H and the vector b
H = spalloc(length(g.x), length(g.x), nnz);
b = zeros(length(g.x), 1);

needToAddPrior = true;

% compute the addend term to H and b for each of our constraints
disp('linearize and build system');
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.fromIdx:edge.fromIdx+2);  % the first robot pose
    x2 = g.x(edge.toIdx:edge.toIdx+2);      % the second robot pose

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_pose_constraint(x1, x2, edge.measurement);


    % TODO: compute and add the term to H and b
    % Set indexes of the i'th pose in the graph
    i = edge.fromIdx:edge.fromIdx+2;
    % Set indexes of the j'th pose in the graph, where j=i+1
    j = edge.toIdx:edge.toIdx+2;

    %Set the information matrix
    omega = edge.information;

    % Compute the coefficent vector for both first and second pose
    b(i) = b(i) + (e'*omega*A)';
    b(j) = b(j) + (e'*omega*B)';

    % Update the system matrix
    H(i,i) = H(i,i) + A'*omega*A;
    H(i,j) = H(i,j) + A'*omega*B;
    H(j,i) = H(j,i) + B'*omega*A;
    H(j,j) = H(j,j) + B'*omega*B;

    if (needToAddPrior)
      % TODO: add the prior for one pose of this edge
      % This fixes one node to remain at its current location
      H(1:3, 1:3) = H(1:3, 1:3) + eye(3, 3);

      needToAddPrior = false;
    end

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose and the landmark in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    x2 = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_landmark_constraint(x1, x2, edge.measurement);


    % TODO: compute and add the term to H and b
    % Set indexes of the i'th pose in the graph
    i = edge.fromIdx:edge.fromIdx + 2;
    % Set indexes of the j'th pose in the graph, where j=i+1
    j = edge.toIdx:edge.toIdx + 1;

    %Set the information matrix
    omega = edge.information;

    % Compute the coefficent vector for both first and second pose
    b(i) = b(i) + (e'*omega*A)';
    b(j) = b(j) + (e'*omega*B)';

    % Update the system matrix
    H(i,i) = H(i,i) + A'*omega*A;
    H(i,j) = H(i,j) + A'*omega*B;
    H(j,i) = H(j,i) + B'*omega*A;
    H(j,j) = H(j,j) + B'*omega*B;

  end
end

disp('solving system');

% TODO: solve the linear system, whereas the solution should be stored in dx
% Remember to use the backslash operator instead of inverting H
dx = -H\b;

end
