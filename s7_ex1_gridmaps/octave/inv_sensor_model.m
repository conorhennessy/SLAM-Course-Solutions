function [mapUpdate, robPoseMapFrame, laserEndPntsMapFrame] = inv_sensor_model(map, scan, robPose, gridSize, offset, probOcc, probFree)
% Compute the log odds values that should be added to the map based on the inverse sensor model
% of a laser range finder.

% map is the matrix containing the occupancy values (IN LOG ODDS) of each cell in the map.
% scan is a laser scan made at this time step. Contains the range readings of each laser beam.
% robPose is the robot pose in the world coordinates frame.
% gridSize is the size of each grid in meters.
% offset = [offsetX; offsetY] is the offset that needs to be subtracted from a point
% when converting to map coordinates.
% probOcc is the probability that a cell is occupied by an obstacle given that a
% laser beam endpoint hit that cell.
% probFree is the probability that a cell is occupied given that a laser beam passed through it.

% mapUpdate is a matrix of the same size as map. It has the log odds values that need to be added for the cells
% affected by the current laser scan. All unaffected cells should be zeros.
% robPoseMapFrame is the pose of the robot in the map coordinates frame.
% laserEndPntsMapFrame are map coordinates of the endpoints of each laser beam (also used for visualization purposes).

% Initialize mapUpdate.
mapUpdate = zeros(size(map));

% Robot pose as a homogeneous transformation matrix.
robTrans = v2t(robPose);

% TODO: compute robPoseMapFrame. Use your world_to_map_coordinates implementation.


% Compute the Cartesian coordinates of the laser beam endpoints.
% Set the third argument to 'true' to use only half the beams for speeding up the algorithm when debugging.
laserEndPnts = robotlaser_as_cartesian(scan, 30, false);

% Compute the endpoints of the laser beams in the world coordinates frame.
laserEndPnts = robTrans*laserEndPnts;
% TODO: compute laserEndPntsMapFrame from laserEndPnts. Use your world_to_map_coordinates implementation.


% freeCells are the map coordinates of the cells through which the laser beams pass.
freeCells = [];

% Iterate over each laser beam and compute freeCells.
% Use the bresenham method available to you in tools for computing the X and Y
% coordinates of the points that lie on a line.
% Example use for a line between points p1 and p2:
% [X,Y] = bresenham(map,[p1_x, p1_y; p2_x, p2_y]);
% You only need the X and Y outputs of this function.
for sc=1:columns(laserEndPntsMapFrame)
        %TODO: compute the XY map coordinates of the free cells along the laser beam ending in laserEndPntsMapFrame(:,sc)


        %TODO: add them to freeCells


endfor


%TODO: update the log odds values in mapUpdate for each free cell according to probFree.


%TODO: update the log odds values in mapUpdate for each laser endpoint according to probOcc.


end
