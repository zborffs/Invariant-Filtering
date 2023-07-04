function G = exp_se2(g)

theta = g(3);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

if abs(theta) < 0.1
    % theta is small so calculate A and B using Taylor series
    A = 1 - theta^2 / 6 * (1 - theta^2 / 20 * (1 - theta^2 / 42));
    B = theta / 2 * (1 - theta^2 / 12 * (1 - theta^2/30 * (1-theta^2/56)));
else
    A = sin(theta) / theta;
    B = (1 - cos(theta)) / theta;
end

V = [A -B; B A];

G = [R V * g(1:2); zeros(1,2) 1];

end