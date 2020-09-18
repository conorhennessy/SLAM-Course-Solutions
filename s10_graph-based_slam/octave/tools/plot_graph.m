% plot a 2D SLAM graph
function plot_graph(g, iteration = -1)

clf;
hold on;

[p, l] = get_poses_landmarks(g);

if (length(l) > 0)
  landmarkIdxX = l+1;
  landmarkIdxY = l+2;
  plot(g.x(landmarkIdxX), g.x(landmarkIdxY), '.or', 'markersize', 4);
end

if (length(p) > 0)
  pIdxX = p+1;
  pIdxY = p+2;
  plot(g.x(pIdxX), g.x(pIdxY), '.xb', 'markersize', 4);
end

% draw line segments???
if 0
  poseEdgesP1 = [];
  poseEdgesP2 = [];
  landmarkEdgesP1 = [];
  landmarkEdgesP2 = [];
  for eid = 1:length(g.edges)
    edge = g.edges(eid);
    if (strcmp(edge.type, 'P') != 0)
      poseEdgesP1 = [poseEdgesP1, g.x(edge.fromIdx:edge.fromIdx+1)];
      poseEdgesP2 = [poseEdgesP2, g.x(edge.toIdx:edge.toIdx+1)];
    elseif (strcmp(edge.type, 'L') != 0)
      landmarkEdgesP1 = [landmarkEdgesP1, g.x(edge.fromIdx:edge.fromIdx+1)];
      landmarkEdgesP2 = [landmarkEdgesP2, g.x(edge.toIdx:edge.toIdx+1)];
    end
  end
  
  linespointx = [poseEdgesP1(1,:); poseEdgesP2(1,:)];
  linespointy = [poseEdgesP1(2,:); poseEdgesP2(2,:)];

  plot(linespointx, linespointy, "r");
end

%plot(poseEdgesP1(1,:), poseEdgesP1(2,:), "r");

%if (columns(poseEdgesP1) > 0)
%end
%if (columns(landmarkEdges) > 0)
%end

hold off;

figure(1, "visible", "on");
drawnow;
%pause(0.1);
if (iteration >= 0)
  filename = sprintf('../plots/lsslam_%03d.png', iteration);
  print(filename, '-dpng');
end


end
