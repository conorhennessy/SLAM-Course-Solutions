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

% the cummulative sum
cs = cumsum(w);
weightSum = cs(length(cs));

% initialize the step and the current position on the roulette wheel
step = weightSum / numParticles;
position = unifrnd(0, weightSum);
idx = 1;

% walk along the wheel to select the particles
for i = 1:numParticles
  position += step;
  if (position > weightSum)
    position -= weightSum;
    idx = 1;
  end
  while (position > cs(idx))
    idx++;
  end
  newParticles(i) = particles(idx);
  newParticles(i).weight = 1/numParticles;
end

end
