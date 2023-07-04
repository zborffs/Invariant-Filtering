function tau = vee_se2(tau_hat)
    tau = zeros(3,1);
    tau(1) = tau_hat(1,3);
    tau(2) = tau_hat(2,3);
    tau(3) = tau_hat(2,1);
end