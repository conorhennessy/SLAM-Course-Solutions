close all;
clear all;
clc;

%% Book Cover feature detection by SURF feature detection
% Refrence: https://uk.mathworks.com/videos/computer-vision-made-easy-81802.html

%import image
lostBook = imread('lostBook3.jpg');
%Turn image into greyscale image
I = rgb2gray(lostBook);

% Find SURF features in image and show strongest 10 points plotted on image
points = detectSURFFeatures(I);
imshow(I); hold on;
plot(points.selectStrongest(10));

% Visualise strongest SURF Points from image
VisualizeStrongestSURFPoints(10, points, I);

function VisualizeStrongestSURFPoints(nPts, points, I)
figure;
title('Strongest Features');
% Select strongest nPts
pts = points.selectStrongest(nPts);
for i=1:nPts
    scale = pts(i).Scale;
    % Crop region around feature based on SURF Scale
    image = imcrop(I,[pts(i).Location-10*scale 20*scale 20*scale]);
    subplot(2,nPts/2,i);
    imshow(image);
    hold on;
    rectangle('Position',[5*scale 5*scale 10*scale 10*scale],'Curvature',1,'EdgeColor','g');
end
end
