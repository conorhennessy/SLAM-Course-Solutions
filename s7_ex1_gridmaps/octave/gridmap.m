addpath('tools')
more off
close all
clear all

% Load laser scans and robot poses.
load("../data/laser")
%laser = read_robotlaser('../data/csail.log');

% Extract robot poses: Nx3 matrix where each row is in the form: [x y theta]
poses = [laser.pose];
poses = reshape(poses,3,size(poses,2)/3)';

% Initial cell occupancy probability.
prior = 0.50;
% Probabilities related to the laser range finder sensor model.
probOcc = 0.9;
probFree = 0.35;

% Map grid size in meters. Decrease for better resolution.
gridSize = 0.1;
% gridSize = 0.1 for making the video

% Set up map boundaries and initialize map.
border = 30;
robXMin = min(poses(:,1));
robXMax = max(poses(:,1));
robYMin = min(poses(:,2));
robYMax = max(poses(:,2));
mapBox = [robXMin-border robXMax+border robYMin-border robYMax+border];
offsetX = mapBox(1);
offsetY = mapBox(3);
mapSizeMeters = [mapBox(2)-offsetX mapBox(4)-offsetY];
mapSize = ceil([mapSizeMeters/gridSize]);

% Used when updating the map. Assumes that prob_to_log_odds.m
% has been implemented correctly.
logOddsPrior = prob_to_log_odds(prior);

% The occupancy value of each cell in the map is initialized with the prior.
map = logOddsPrior*ones(mapSize);
disp('Map initialized. Map size:'), disp(size(map))

% Map offset used when converting from world to map coordinates.
offset = [offsetX; offsetY];

% Main loop for updating map cells.
% You can also take every other point when debugging to speed up the loop (t=1:2:size(poses,1))
for(t=1:size(poses,1))
	t
	% Robot pose at time t.
	robPose = [poses(t,1);poses(t,2);poses(t,3)];
	
	% Laser scan made at time t.
	sc = laser(1,t);
	% Compute the mapUpdate, which contains the log odds values to add to the map.
	[mapUpdate, robPoseMapFrame, laserEndPntsMapFrame] = inv_sensor_model(map, sc, robPose, gridSize, offset, probOcc, probFree);

	mapUpdate -= logOddsPrior*ones(size(map));
	% Update the occupancy values of the affected cells.
	map += mapUpdate;
	
	% Plot current map and robot trajectory so far.
        plot_map(map, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t);
endfor
