function adjoint = adjoint_se2(chi)

R = chi(1:2,1:2);
t = chi(1:2,3);
cross_1 = [0 -1; 1 0];

adjoint = [R -cross_1 * t; zeros(1,2) 1];

end