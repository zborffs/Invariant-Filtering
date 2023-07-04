function Jr = Jr_se2(tau)

    x = tau(1);
    y = tau(2);
    theta = tau(3);

    if abs(theta) < 0.01
        a = 1/2 * (1 - theta^2/12 * (1 - theta^2/30 * (1 - theta^2 /56))); % (1-cos(theta))/theta^2
        b = 1 - theta^2/6 * (1 - theta^2 / 20 * (1 - theta^2/42)); % sin(theta)/theta
        c = theta * (1/6 * (1 - theta^2/20 * (1-theta^2/42 * (1-theta^2/72))));
        A = [
            c -a;
            a c
        ] * [x;y];


        Jr = [
            b a A(1);
            -a b A(2);
            0 0 1
        ];
    else
        A = [
            theta - sin(theta) -1 + cos(theta);
            1-cos(theta) theta - sin(theta)
        ] * [x;y];
        A = A / theta^2;
        Jr = [
            sin(theta)/theta (1-cos(theta))/theta A(1);
            -(1-cos(theta))/theta sin(theta)/theta A(2);
            0 0 1
        ];
    end
end