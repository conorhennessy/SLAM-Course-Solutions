#computes the pose vector v from an homogeneous transform A
function v=t2v(A)
  v = [A(1:2,3); atan2(A(2,1),A(1,1))];
end
