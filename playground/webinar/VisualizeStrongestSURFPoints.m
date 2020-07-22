%% Visualize regions around strongest SURF features
% Copyright 2014 The MathWorks, Inc.

function VisualizeStrongestSURFPoints(nPts,points,I)

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