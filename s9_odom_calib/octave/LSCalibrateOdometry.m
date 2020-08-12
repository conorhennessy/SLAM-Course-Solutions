more off;
clear all;
close all;

% add tools directory
addpath('tools')

% load the odometry measurements
load ../data/odom_motions

% the motions as they are estimated by scan-matching
load ../data/scanmatched_motions

% create our measurements vector z
z = [scanmatched_motions odom_motions];

% perform the calibration
X = ls_calibrate_odometry(z);
disp('calibration result'); disp(X);

% apply the estimated calibration parameters
calibrated_motions = apply_odometry_correction(X, odom_motions);

% compute the current odometry trajectory, the scanmatch result, and the calibrated odom
odom_trajectory = compute_trajectory(odom_motions);
scanmatch_trajectory = compute_trajectory(scanmatched_motions);
calibrated_trajectory = compute_trajectory(calibrated_motions);

% plot the trajectories
plot(
  odom_trajectory(:,1), odom_trajectory(:,2), ";Uncalibrated Odometry;",
  scanmatch_trajectory(:,1), scanmatch_trajectory(:,2), ";Scan-Matching;",
  calibrated_trajectory(:,1), calibrated_trajectory(:,2), ";Calibrated Odometry;");
print -dpng "../plots/odometry-calibration.png"
