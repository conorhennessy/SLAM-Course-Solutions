% compute the expected measurement for a landmark
% and the Jacobian with respect to the landmark
function [h, H] = measurement_model(particle, z)

% extract the id of the landmark
landmarkId = z.id;
% two 2D vector for the position (x,y) of the observed landmark
landmarkPos = particle.landmarks(landmarkId).mu;

% TODO: use the current state of the particle to predict the measurment
landmarkX = landmarkPos(1);
landmarkY = landmarkPos(2);
expectedRange = sqrt((landmarkX - particle.pose(1))^2 + (landmarkY - particle.pose(2))^2);
expectedBearing = normalize_angle(atan2(landmarkY-particle.pose(2), landmarkX-particle.pose(1)) - particle.pose(3));
h = [expectedRange; expectedBearing];

% TODO: Compute the Jacobian H of the measurement function h wrt the landmark location
H = zeros(2,2);
H(1,1) = (landmarkX - particle.pose(1))/expectedRange;
H(1,2) = (landmarkY - particle.pose(2))/expectedRange;
H(2,1) = (particle.pose(2) - landmarkY)/(expectedRange^2);
H(2,2) = (landmarkX - particle.pose(1))/(expectedRange^2);

end
