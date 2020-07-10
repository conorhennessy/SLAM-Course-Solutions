%% Book Cover feature detection by SURF feature detection, extraction and matching
% With RANSAC to filter out outliers. By est. of geometric transformation
% and fundalmental matrix.
% RANSAC; Random Sample Consensus. An iterative est. of parameters to a
% mathamatical model from a set of observed data that contains outliers.
% Refrence: https://uk.mathworks.com/videos/computer-vision-made-easy-81802.html

close all;
clc;

%% Read image
I1 = imread('BookSpine.jpg');
figure; imshow(I1); title('Obj');
I2 = imread('bookshelf.jpg');
figure; imshow(I2); title('Scene');

% Define location of object in I1
boxPolygon = [1, 1;...                           % top-left
             size(I1, 2), 1;...                  % top-right
             size(I1, 2), size(I1, 1);...        % bottom-right
             1, size(I1, 1);...                  % bottom-left
             1, 1];                               % top-left again to close the polygon

%% Detect Features
points1 = detectSURFFeatures(rgb2gray(I1));
points2 = detectSURFFeatures(rgb2gray(I2));


%% Extract Features
[feats1, validpts1] = extractFeatures(rgb2gray(I1), points1);
[feats2, validpts2] = extractFeatures(rgb2gray(I2), points2);

%%Display Features
figure; imshow(I1); hold on; plot(validpts1, 'showOrientation', true);
title('Detected Features');

%% Match Features
index_pairs = matchFeatures(feats1, feats2, 'Prenormalized', true);
matched_pts1 = validpts1(index_pairs(:, 1));
matched_pts2 = validpts2(index_pairs(:, 2));
%% Remove outliers while est. geometric transform using RANSAC
[transform, inlierPoints1, inlierPoints2] = estimateGeometricTransform(matched_pts1, matched_pts2, 'affine');
figure; showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2, 'montage');
title('Filtered Matches');

%% Use est. transform to locate the obj.
newBoxPolygon = transformPointsForward(transform, boxPolygon);
figure; imshow(I2);
hold on; 
line(newBoxPolygon(:, 1), newBoxPolygon(:, 2), 'Color', 'g', 'LineWidth', 5);
title('Detected obj.');