function g = log_se2(G)

theta = atan2(G(2,1), G(1,1));

if abs(theta) < 0.1
    % theta is small so calculate A and B using Taylor series
    A = 1 - theta^2 / 6 * (1 - theta^2 / 20 * (1 - theta^2 / 42));
    B = theta / 2 * (1 - theta^2 / 12 * (1 - theta^2/30 * (1-theta^2/56)));
else
    A = sin(theta) / theta;
    B = (1 - cos(theta)) / theta;
end

V_inv = 1 / (A^2 + B^2) * [A B; -B A];

g = [V_inv * G(1:2,3); theta];

end