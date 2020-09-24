function p = log_odds_to_prob(l)
% Convert log odds l to the corresponding probability values p.
% l could be a scalar or a matrix.

% TODO: compute p.
p=[];
for i = 1:size(l,1)
    for j = 1:size(l,2)
	p(i,j) = 1 - 1/(1 + exp(l(i,j)));
    endfor
endfor

% p = eye() - (1 ./ (1+exp(l)));

end
