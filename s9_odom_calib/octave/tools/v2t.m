#computes the homogeneous transform matrix A of the pose vector v
function A=v2t(v)
  	c=cos(v(3));
  	s=sin(v(3));
	A=[c, -s, v(1);
	s,  c, v(2);
	0   0  1  ];
end
