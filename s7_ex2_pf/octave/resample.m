% resample the set of particles.
% A particle has a probability proportional to its weight to get
% selected. A good option for such a resampling method is the so-called low
% variance sampling, Probabilistic Robotics pg. 109
function newParticles = resample(particles)

numParticles = length(particles);

w = [particles.weight];

% normalize the weight
w = w / sum(w);

% consider number of effective particles, to decide whether to resample or not
useNeff = false;
%useNeff = true;
if useNeff
  neff = 1. / sum(w.^2);
  neff
  if neff > 0.5*numParticles
    newParticles = particles;
    for i = 1:numParticles
      newParticles(i).weight = w(i);
    end
    return;
  end
end

newParticles = struct;

% TODO: implement the low variance re-sampling
% This corresponds to slide 18 of Particle Filter lecture. 26:00 in SLAM Course - 11 video
%% Note: low variance re-sampling, also called Stochastic Universal Sampling
%% O(J)
%% This sampling is in linear time. If all samples all have the same weight,
%% this guarentees we get the same samples every time. Thus low variance. 

% the cumulative sum
c = w(1);

% initialize the step and the current position on the roulette wheel
r = rand(1) * numParticles;
i = 1;

% walk along the wheel to select the particles
for j = 1:newParticles
  U = r + (j-1) * inv(newParticles);
  while U > c
    i = i + 1;
    c = c + w(i);
  endwhile
  newParticles(j) = newParticles(i);
endfor

end
