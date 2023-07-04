function tau_hat = hat_se2(tau)
    G1 = [0 0 1; 0 0 0; 0 0 0];
    G2 = [0 0 0; 0 0 1; 0 0 0];
    G3 = [0 -1 0; 1 0 0; 0 0 0];
    tau_hat = tau(1) * G1 + tau(2) * G2 + tau(3) * G3;
end