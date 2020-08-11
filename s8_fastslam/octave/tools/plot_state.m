function plot_state(particles, landmarks, timestep, z, window)
    % Visualizes the state of the FastSLAM algorithm.
    %
    % The resulting plot displays the following information:
    % - map ground truth (black +'s)
    % - currently best particle (red)
    % - particle set in green
    % - current landmark pose estimates (blue)
    % - visualization of the observations made at this time step (line between robot and landmark)

    clf;
    hold on
    grid("on")

    L = struct2cell(landmarks); 
    plot(cell2mat(L(2,:)), cell2mat(L(3,:)), 'k+', 'markersize', 10, 'linewidth', 5);
    
    % Plot the particles
    ppos = [particles.pose];
    plot(ppos(1,:), ppos(2,:), 'g.');

    % determine the currently best particle
    [bestWeight, bestParticleIdx] = max([particles.weight]);

    % draw the landmark locations along with the ellipsoids
    for(i=1:length(particles(bestParticleIdx).landmarks))
        if(particles(bestParticleIdx).landmarks(i).observed)
          l = particles(bestParticleIdx).landmarks(i).mu;
          plot(l(1), l(2), 'bo', 'markersize', 3);
          drawprobellipse(l, particles(bestParticleIdx).landmarks(i).sigma, 0.95, 'b');
        end
    end

    % draw the observations
    for(i=1:size(z,2))
      l = particles(bestParticleIdx).landmarks(z(i).id).mu;
      line([particles(bestParticleIdx).pose(1), l(1)],[particles(bestParticleIdx).pose(2), l(2)], 'color', 'k', 'linewidth', 1);
    end

    % draw the trajectory as estimated by the currently best particle
    trajectory = cell2mat(particles(bestParticleIdx).history);
    line(trajectory(1,:), trajectory(2, :), 'color', 'r', 'linewidth', 3);

    drawrobot(particles(bestParticleIdx).pose, 'r', 3, 0.3, 0.3);
    xlim([-2, 12])
    ylim([-2, 12])

    hold off

    % dump to a file or show the window
    if window
      figure(1, "visible", "on");
      drawnow;
      pause(0.1);
    else
      figure(1, "visible", "off");
      filename = sprintf('../plots/fastslam_%03d.png', timestep);
      print(filename, '-dpng');
    end

end
