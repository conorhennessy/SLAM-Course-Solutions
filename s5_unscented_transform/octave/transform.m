function [points] = transform(points)
% This function applies a transformation to a set of points.
% Each column in points is one point, i.e. points = [[x1; y1], [x2;y2], ...]
% Select which function you want to use by uncommenting it
% (deleting the corresponding %{...%}) while keeping all other functions commented.


%%%%%
% Function 1 (linear)
% Applies a translation to [x; y]
points(1,:) = points(1,:) + 1;
points(2,:) = points(2,:) + 2;
%%%%%

%%%%%
%{
% Function 2 (nonlinear)
% Computes the polar coordinates corresponding to [x; y]
x = points(1,:);
y = points(2,:);
r = sqrt(sum([x.*x; y.*y]));
theta = atan2(y,x);
points = [r;theta];
%}
%%%%%

%%%%%
%{
% Function 3 (nonlinear)
points(1,:) = points(1,:).*cos(points(1,:)).*sin(points(1,:));
points(2,:) = points(2,:).*cos(points(2,:)).*sin(points(2,:));
%}
%%%%%
