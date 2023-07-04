% generate fake data simulating a rigid body randomly moving around
rng(0);
n = 60;%1000; % number of samples

% yeah
drawArrow = @(x,y,theta,color) quiver(x,y,cos(theta),sin(theta),0,color);

% state is defined by x = [position; orientation; linear velocity; ang. velocity]
r_PO_I_0 = zeros(2,1); % position of P relative to O expresesd in I frame (initial)
R_PO_I_0 = [1 0; 0 1];% eye(2,2); % orientation of P relative to O expressed in I frame (initial)
v_PO_I_0 = zeros(2,1); % inertial linear velocity of P relative to O expressed in I frame (initial)
omega_PO_I_0 = zeros(1,1); % inertial angular velocity of P relative to O expressed in I frame (initial)

% initialize state "vector" histories
r_PO_I = zeros(2,n); r_PO_I(:,1) = r_PO_I_0;
R_PO_I = zeros(2,2,n); R_PO_I(:,:,1) = R_PO_I_0;
v_PO_I = zeros(2,n); v_PO_I(:,1) = v_PO_I_0;
omega_PO_I = zeros(1,n); omega_PO_I_0(:,1) = omega_PO_I_0;
ytilde = zeros(5,n);

% simulate the system forward in time according to assumed dynamics
for ii = 2:n
    % random rates based on body frame
    u = 1.0;
    v = 0.0;
    omega = 2*pi/n;

    % 
    G = [R_PO_I(:,:,ii-1) r_PO_I(:,ii-1); zeros(1,2) 1];

    % measurements
    if rand < 0.4
        gps_meas = [missing; missing; missing];
    else
        gps_meas = G * [0;0;1] + [1.0; -0.78; 0]; % small GPS bias
    end
    
    Q = [
        0.9 0 0 0 0; 
        0 1.0 0 0 0;
        0 0 0.1 0 0;
        0 0 0 0.1 0;
        0 0 0 0 0.1
    ];
    noise = Q * randn(5,1);
    ytilde(:,ii) = [gps_meas(1:2); u; v; omega] + noise;

    % propogate
    G_new = G * exp_se2([u;v;omega]);
    g_new = [u;v;omega];

    r_PO_I(:,ii) = G_new(1:2,3);
    R_PO_I(:,:,ii) = G_new(1:2,1:2);
    v_PO_I(:,ii) = [u;v];
    omega_PO_I(ii) = omega;
end

% plot system
figure(1);
for ii = 1:size(r_PO_I,2)
    drawArrow(r_PO_I(1,ii), r_PO_I(2,ii), atan2(R_PO_I(2,1,ii), R_PO_I(1,1,ii)),'k'); hold on;
end
scatter(ytilde(1,:), ytilde(2,:), 'r+'); grid on;
hold off;

figure(2);
subplot(5,1,1);
plot(ytilde(1,:), 'r+'); grid on; hold on;
subplot(5,1,2);
plot(ytilde(2,:), 'r+'); grid on;
subplot(5,1,3);
plot(ytilde(3,:), 'k-'); grid on;
subplot(5,1,4);
plot(ytilde(4,:), 'k-'); grid on;
subplot(5,1,5);
plot(ytilde(5,:), 'k-'); grid on;
hold off;


%% just propogation
% state is defined by x = [position; orientation; linear velocity; ang. velocity]
% filtered_r_PO_I_0 = zeros(2,1); % position of P relative to O expresesd in I frame (initial)
filtered_r_PO_I_0 = [0.7;-1.0]; % position of P relative to O expresesd in I frame (initial)
% filtered_R_PO_I_0 = [1 0; 0 1];% eye(2,2); % orientation of P relative to O expressed in I frame (initial)
% filtered_R_PO_I_0 = [cos(pi/4) -sin(pi/4); sin(pi/4) cos(pi/4)];% eye(2,2); % orientation of P relative to O expressed in I frame (initial)
% filtered_R_PO_I_0 = [cos(pi/2) -sin(pi/2); sin(pi/2) cos(pi/2)];% eye(2,2); % orientation of P relative to O expressed in I frame (initial)
% filtered_R_PO_I_0 = [cos(pi) -sin(pi); sin(pi) cos(pi)];% eye(2,2); % orientation of P relative to O expressed in I frame (initial)
filtered_R_PO_I_0 = [cos(-pi/2) -sin(-pi/2); sin(-pi/2) cos(-pi/2)];% eye(2,2); % orientation of P relative to O expressed in I frame (initial)
filtered_v_PO_I_0 = zeros(2,1); % inertial linear velocity of P relative to O expressed in I frame (initial)
filtered_omega_PO_I_0 = zeros(1,1); % inertial angular velocity of P relative to O expressed in I frame (initial)

% initialize state "vector" histories
filtered_r_PO_I = zeros(2,n); filtered_r_PO_I(:,1) = filtered_r_PO_I_0;
filtered_R_PO_I = zeros(2,2,n); filtered_R_PO_I(:,:,1) = filtered_R_PO_I_0;

% we are measuring (x,y,u,v,omega)!
z = nan(2,n);
P = 1e1 * eye(3,3); P(3,3) = 1000;
N = 1 * eye(2);
W = 1 * eye(3);
for ii = 2:n
    % random rates based on body frame
    u = ytilde(3,ii-1);
    v = ytilde(4,ii-1);
    omega = ytilde(5,ii-1);
    tau = [u;v;omega];

    % 
    G = [filtered_R_PO_I(:,:,ii-1) filtered_r_PO_I(:,ii-1); zeros(1,2) 1];

    % propogate
    G_new = G * exp_se2(tau);

    % compute jacobians
    F = inv(adjoint_se2(exp_se2(tau)));
    Gamma = Jr_se2(tau);
    P = F * P * F' + Gamma * 0.01*eye(3) * Gamma';

    % yep
    if ~all(isnan(ytilde(1:2,ii)))
%         ynew = inv(G_new) * [0;0;1];
        ynew = G_new * [0;0;1];
        z(:,ii) = ytilde(1:2,ii) - ynew(1:2);
%         H = -[eye(2) G_new(1:2,1:2)' * [0 -1; 1 0] * -G_new(1:2,3)];
        H = [G_new(1:2,1:2) G_new(1:2,1:2) * [0 -1; 1 0] * [0;0]];
        Z = H * P * H' + N;
        K = P * H' * inv(Z);

        G_new = G_new * exp_se2(K * z(:,ii));
        P = P - K * Z * K';
    end

    filtered_r_PO_I(:,ii) = G_new(1:2,3);
    filtered_R_PO_I(:,:,ii) = G_new(1:2,1:2);
end

% plot system
figure(3);
for ii = 1:size(filtered_r_PO_I,2)
    drawArrow(filtered_r_PO_I(1,ii), filtered_r_PO_I(2,ii), atan2(filtered_R_PO_I(2,1,ii), filtered_R_PO_I(1,1,ii)), 'k'); hold on;
    drawArrow(r_PO_I(1,ii), r_PO_I(2,ii), atan2(R_PO_I(2,1,ii), R_PO_I(1,1,ii)), 'g'); hold on;
end
grid on;
scatter(ytilde(1,:), ytilde(2,:), 'r+'); grid on;
hold off;

% figure(4);
% subplot(2,1,1)
% plot(ytilde(1,:) - filtered_r_PO_I(1,ii))
% subplot(2,1,2)
% plot(ytilde(1,:) - filtered_r_PO_I(1,ii))
