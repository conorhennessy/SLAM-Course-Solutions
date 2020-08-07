#computes the pose vector v from an homogeneous transform A
function v=t2v(A)
	v(1:2, 1)=A(1:2,3);
	v(3,1)=atan2(A(2,1),A(1,1));
end
