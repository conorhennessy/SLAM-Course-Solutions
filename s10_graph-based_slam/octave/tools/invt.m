% inverts a homogenous transform
function A = invt(m)
  A = [m(1:2, 1:2)' [0 0]'; [0 0 1]];
  A(1:2, 3) = -A(1:2, 1:2) * m(1:2, 3);
end
