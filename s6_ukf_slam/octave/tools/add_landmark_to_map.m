% Add a landmark to the UKF.
% We have to compute the uncertainty of the landmark given the current state
% (and its uncertainty) of the newly observed landmark. To this end, we also
% employ the unscented transform to propagate Q (sensor noise) through the
% current state

function [mu, sigma, map] = add_landmark_to_map(mu, sigma, z, map, Q);

% For computing sigma
global scale;

landmarkId = z.id;

%add landmark to the map
map = [map; landmarkId];
% TODO: Initialize its pose according to the measurement and add it to mu

% Append the measurement to the state vector
mu = [mu; z.range(); z.bearing()];
% Initialize its uncertainty and add it to sigma
sigma = blkdiag(sigma, Q);

% Transform from [range, bearing] to the x/y location of the landmark
% This operation intializes the uncertainty in the position of the landmark
% Sample sigma points
sig_pnts_new = compute_sigma_points(mu, sigma);
% Normalize!
sig_pnts_new(3,:) = normalize_angle(sig_pnts_new(3,:));
% Compute the xy location of the new landmark according to each sigma point
newX = sig_pnts_new(1,:) + sig_pnts_new(end-1,:).*cos(sig_pnts_new(3,:) + sig_pnts_new(end,:));
newY = sig_pnts_new(2,:) + sig_pnts_new(end-1,:).*sin(sig_pnts_new(3,:) + sig_pnts_new(end,:));
% The last 2 components of the sigma points can now be replaced by the xy pose of the landmark
sig_pnts_new(end-1,:) = newX;
sig_pnts_new(end,:) = newY;

% Recover mu and sigma
n = length(mu);
lambda = scale - n;
w0 = lambda/scale;
wm = [w0, repmat(1/(2*scale),1,2*n)];
wc = wm;
% Theta should be recovered by summing up the sines and cosines
cosines = sum(cos(sig_pnts_new(3,:)).*wm);
sines = sum(sin(sig_pnts_new(3,:)).*wm);
% recompute the angle and normalize it
mu_theta = normalize_angle(atan2(sines, cosines));
mu = sum(sig_pnts_new .* repmat(wm, rows(sig_pnts_new), 1), 2);
mu(3) = mu_theta;

diff = sig_pnts_new - repmat(mu,1,size(sig_pnts_new,2));
% Normalize!
diff(3,:) = normalize_angle(diff(3,:));
sigma = (repmat(wc, rows(diff), 1) .* diff) * diff';

end
