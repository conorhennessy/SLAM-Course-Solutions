function plot_state(particles, timestep)
    % Visualizes the state of the particles
    
    clf;
    hold on
    grid("on")

    % Plot the particles
    ppos = [particles.pose];
    plot(ppos(1,:), ppos(2,:), 'g.', 'markersize', 10, 'linewidth', 3.5);

    xlim([-2, 12])
    ylim([-2, 12])
    hold off

    % dump to a file or show the window
    %window = true;
    window = false;
    if window
      figure(1, "visible", "on");
      drawnow;
      pause(0.5);
    else
      figure(1, "visible", "off");
      filename = sprintf('../plots/pf_%03d.png', timestep);
      print(filename, '-dpng');
    end

end
