%% Book Cover feature detection by SURF feature detection, extraction and matching
% Refrence: https://uk.mathworks.com/videos/computer-vision-made-easy-81802.html
close all;
clc;

%% Read image
I1 = imread('LostBook3.jpg');
figure; imshow(I1); title('Obj');
I2 = imread('ClutteredScene3.jpg');
figure; imshow(I2); title('Scene');

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
figure; showMatchedFeatures(I1, I2, matched_pts1, matched_pts2, 'montage');
title('Initial Matches');
