function plot_state(mu, sigma, landmarks, timestep, observedLandmarks, z, window)
    % Visualizes the state of the EKF SLAM algorithm.
    %
    % The resulting plot displays the following information:
    % - map ground truth (black +'s)
    % - current robot pose estimate (red)
    % - current landmark pose estimates (blue)
    % - visualization of the observations made at this time step (line between robot and landmark)

    clf;
    hold on
    grid("on")
    L = struct2cell(landmarks); 
    drawprobellipse(mu(1:3), sigma(1:3,1:3), 0.6, 'r');
    plot(cell2mat(L(2,:)), cell2mat(L(3,:)), 'k+', 'markersize', 10, 'linewidth', 5);
    for(i=1:length(observedLandmarks))
	if(observedLandmarks(i))
	    plot(mu(2*i+ 2),mu(2*i+ 3), 'bo', 'markersize', 10, 'linewidth', 5)
   	    drawprobellipse(mu(2*i+ 2:2*i+ 3), sigma(2*i+ 2:2*i+ 3,2*i+ 2:2*i+ 3), 0.6, 'b');
	endif
    endfor

    for(i=1:size(z,2))
	mX = mu(2*z(i).id+2);
	mY = mu(2*z(i).id+3);
    	line([mu(1), mX],[mu(2), mY], 'color', 'k', 'linewidth', 1);
    endfor

    drawrobot(mu(1:3), 'r', 3, 0.3, 0.3);
    xlim([-2, 12])
    ylim([-2, 12])
    hold off

    if window
      figure(1, "visible", "on");
      drawnow;
      pause(0.1);
    else
      figure(1, "visible", "off");
      filename = sprintf('../plots/ekf_%03d.png', timestep);
      print(filename, '-dpng');
    end
end
