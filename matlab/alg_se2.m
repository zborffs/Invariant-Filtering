function g = alg_se2(v)

g = [0 0 1; 0 0 0; 0 0 0] * v(1) + [0 0 0; 0 0 1; 0 0 0] * v(2) + [0 -1 0; 1 0 0; 0 0 0] * v(3);

end