%% Things to note:
% 1. This script simulates a planar rigid body B in discrete-time.
% 2. Robot's body frame follows forward-left-up (FLU) convention.

%% generate fake data simulating a planar rigid body moving in a circle
rng(0);
% n = 60; % number of samples
n = 60 * 3; % number of samples

% Define this function for drawing an arrow on a plot to represent the
% rigid body; (x,y,theta) are the cartesian coordinates of the particle in
% the inertial frame and theta is the orientation of the rigid body w.r.t.
% the x-axis of the inertial frame.
drawArrow = @(x,y,theta,color) quiver(x, y, cos(theta),sin(theta),0,color);

% Define the initial conditions of the planar rigid body
r_BO_I_0 = zeros(2,1); % position of the body B relative to origin of inertial frame O expresesd in I frame (initial)
theta_0 = 0; % initial orientation of the body B relative to O expressed in I frame (initial)
R_BO_I_0 = [cos(theta_0) -sin(theta_0); sin(theta_0) cos(theta_0)]; % orientation of B relative to O expressed in I frame (initial), expressed as rotation matrix
v_BO_B_0 = zeros(2,1); % inertial linear velocity of P relative to O expressed in B frame (initial)
omega_BO_B_0 = zeros(1,1); % inertial angular velocity of P relative to O expressed in B frame (initial)

% Initialize the variables specifying the history of planar rigid body
% states
r_PO_I = zeros(2,n); r_PO_I(:,1) = r_BO_I_0;
R_PO_I = zeros(2,2,n); R_PO_I(:,:,1) = R_BO_I_0;
v_PO_B = zeros(2,n); v_PO_B(:,1) = v_BO_B_0;
omega_PO_B = zeros(1,n); omega_PO_B(:,1) = omega_BO_B_0;

% Initialize the measurements vector; we will be measuring the position of
% B in I frame in cartesian coordinates (x,y); we will be measuring the
% inertial linear velocity of the rigid body expressed in the body frame
% (u,v) where u is the "forward" component and v is the "left" velocity
% component; finally, we measure the inertial angular velocity of the rigid
% body expressed inthe body frame (omega) where omega is the yaw rate
y_tilde = zeros(5,n);

% Define the measurement noise covariances
% R = [
%     5 0 0 0 0;  % this row is for the x component of the GPS
%     0 5 0 0 0;  % this row is for the y component of the GPS
%     0 0 0.1 0 0; % this row is for the "forward" component of the linear velocity (in robot's body frame)
%     0 0 0 0.1 0; % this row is for the "left" component of the linear velocity (in robot's body frame)
%     0 0 0 0 0.1  % this row is for the yaw rate component of the angular velocity (in robot's body frame)
% ];

R = [
    5 0 0 0 0;  % this row is for the x component of the GPS
    0 5 0 0 0;  % this row is for the y component of the GPS
    0 0 0.01 0 0; % this row is for the "forward" component of the linear velocity (in robot's body frame)
    0 0 0 0.01 0; % this row is for the "left" component of the linear velocity (in robot's body frame)
    0 0 0 0 0.01  % this row is for the yaw rate component of the angular velocity (in robot's body frame)
];

% Simulate the system forward in time according to assumed dynamics
for ii = 2:n
    % Make up a linear/angular velocity vector to command to the planar 
    % ridig body
    u = 0.8; % move forward at 1 m/s
    v = 0.0; % move left at 0 m/s
    if ii < 60
        omega = 2*pi/100; % for the first 500 seconds, yaw counterclockwise at 0.06 rad/s
    else
        omega = 0; % after the first 500 seconds, don't yaw at all
    end

    % Put the state from the previous time-step into a matrix; this matrix
    % is a member of SE(2).
    G = [R_PO_I(:,:,ii-1) r_PO_I(:,ii-1); zeros(1,2) 1];

    % Collect noisy measurements of previous state
    if rand < 0.7 
        % 70% of the time, we won't receive GPS measurements. This mimics
        % random and slightly slower update frequency of GPS measurements
        % compared to odom measurements
        gps_meas = [missing; missing; missing];
    else
        gps_meas = G * [0;0;1] + [0.5; -0.26; 0]; % small GPS bias
    end
    noise = R * randn(5,1);
    y_tilde(:,ii-1) = [gps_meas(1:2); u; v; omega] + noise; % apply Gaussian noise to measurements

    % Propogate the system forward using the assumed dynamics
    G_new = G * exp_se2([u;v;omega]);
    g_new = [u;v;omega];

    % Save the true propogated into their respective variables
    r_PO_I(:,ii) = G_new(1:2,3);
    R_PO_I(:,:,ii) = G_new(1:2,1:2);
    v_PO_B(:,ii) = [u;v];
    omega_PO_B(ii) = omega;
end

%% Plot the state of the system over time
figure(1), clf
scatter(y_tilde(1,:), y_tilde(2,:), 'r+'); grid on; hold on;
for ii = 1:size(r_PO_I,2)
    drawArrow(r_PO_I(1,ii), r_PO_I(2,ii), atan2(R_PO_I(2,1,ii), R_PO_I(1,1,ii)),'k');
end
xlabel("displacement $x$ (m)", "interpreter", "latex")
ylabel("displacement $y$ (m)", "interpreter", "latex")
title("movement of robot over time")
legend("GPS Measurement", "True Pose")
axis image;
hold off;

figure(2);
subplot(5,1,1);
plot(y_tilde(1,:), 'r+'); grid on;
xlabel("time $t$ (s)", "interpreter", "latex")
ylabel("measured position component $x$ (m)", "interpreter", "latex")
subplot(5,1,2);
plot(y_tilde(2,:), 'r+'); grid on;
xlabel("time $t$ (s)", "interpreter", "latex")
ylabel("measured position component $y$ (m)", "interpreter", "latex")
subplot(5,1,3);
plot(y_tilde(3,:), 'k-'); grid on;
xlabel("time $t$ (s)", "interpreter", "latex")
ylabel("measured body-frame forward velocity $u$ (m/s)", "interpreter", "latex")
subplot(5,1,4);
plot(y_tilde(4,:), 'k-'); grid on;
xlabel("time $t$ (s)", "interpreter", "latex")
ylabel("measured body-frame left velocity $v$ (m/s)", "interpreter", "latex")
subplot(5,1,5);
plot(y_tilde(5,:), 'k-'); grid on;
xlabel("time $t$ (s)", "interpreter", "latex")
ylabel("measured yaw rate $\omega$ (rad/s)", "interpreter", "latex")
hold off;


%% Apply InEKF to simulated data.
% state is defined by x = [position; orientation; linear velocity; ang. velocity]
filtered_r_PO_I_0 = [0.7;-1.0]; % position of P relative to O expresesd in I frame (initial)
filtered_theta_0 = -pi/2; % assumed initial orientation w.r.t. map frame x-axis
filtered_R_PO_I_0 = [cos(filtered_theta_0) -sin(filtered_theta_0); sin(filtered_theta_0) cos(filtered_theta_0)];% eye(2,2); % orientation of P relative to O expressed in I frame (initial)

% initialize state "vector" histories
filtered_r_PO_I = zeros(2,n); filtered_r_PO_I(:,1) = filtered_r_PO_I_0;
filtered_R_PO_I = zeros(2,2,n); filtered_R_PO_I(:,:,1) = filtered_R_PO_I_0;

% Initialize empty vectors and other variables for filtering (x,y,u,v,omega)!
z = nan(2,n); % difference between true state and estimated state
P = zeros(3,3,n); P(:,:,1) = 1e1 * eye(3,3); P(3,3,1) = 1000; % estimated state-error covariance over time
N = 1 * eye(2); % assumed process noise covariance (assumed to be Gaussian noise)
% W = 0.01 * eye(3); % assumed measurement noise covariance (assumed to be Gaussian noise)
% W = 1e-6 * eye(3); % assumed measurement noise covariance (assumed to be Gaussian noise)
W = 1e-10 * eye(3); % assumed measurement noise covariance (assumed to be Gaussian noise)

% filter the system
for ii = 2:n
    % Use the measured linear / angular rates as control inputs to system;
    % in reality, these are not "measured" rates. These are actually
    % "estimated rates" because it is assumed that Spot will estimate these
    % based on several sensor measurements and some internal model.
    u = y_tilde(3,ii-1);
    v = y_tilde(4,ii-1);
    omega = y_tilde(5,ii-1);

    % put the measured rates into an element of the Lie algebra
    g = [u;v;omega];

    % Put the estimated orientation and position of the planar rigid body
    % from the previous time step into an element of the Lie group
    G = [filtered_R_PO_I(:,:,ii-1) filtered_r_PO_I(:,ii-1); zeros(1,2) 1];

    % Propogate the system forward according to the assumed model
    G_new = G * exp_se2(g);

    % Compute the necessary Jacobians of the dynamic model to propogate the
    % estimated error covariance forward in time
    F = inv(adjoint_se2(exp_se2(g))); % the Jacobian of the dynamic model w.r.t. the state "G"
    Gamma = Jr_se2(g); % the Jacobian of the dynamic model w.r.t. the control input "g"
    P(:,:,ii) = F * P(:,:,ii-1) * F' + Gamma * W * Gamma'; % propogate the estimated error covariance forward according to the dynamic model

    % If we have received a GPS measurement at this iteration, then apply
    % the "update" steps of the EKF. Otherwise, just skip
    % them, having already propogated the system forward according to the
    % assumed dynamical model
    if ~all(isnan(y_tilde(1:2,ii)))
        % compute the measurement we would have expected assuming
        % measurement model is true and propogated state is true state.
        y_new = G_new * [0;0;1];

        % compute error between the expected measurement and the actual
        % measurement
        z(:,ii) = y_tilde(1:2,ii) - y_new(1:2);

        % Compute the Jacobian of the measurement equation w.r.t. the state
        % G
        H = [G_new(1:2,1:2) G_new(1:2,1:2) * [0 -1; 1 0] * [0;0]];

        % Compute the "innovation covariance"
        Z = H * P(:,:,ii) * H' + N;

        % Compute the Kalman Gain
        K = P(:,:,ii) * H' * inv(Z);

        % Compute the updated estimated state and updated estimated state
        % error covariance
        G_new = G_new * exp_se2(K * z(:,ii));
        P(:,:,ii) = P(:,:,ii) - K * Z * K';
    end

    % save the estimated state and estimated state error covariance
    filtered_r_PO_I(:,ii) = G_new(1:2,3);
    filtered_R_PO_I(:,:,ii) = G_new(1:2,1:2);
end

%% Plot the filtered state and the true state
figure(3), clf;
for ii = 1:size(filtered_r_PO_I,2)
    drawArrow(filtered_r_PO_I(1,ii), filtered_r_PO_I(2,ii), atan2(filtered_R_PO_I(2,1,ii), filtered_R_PO_I(1,1,ii)), 'k'); hold on;
    drawArrow(r_PO_I(1,ii), r_PO_I(2,ii), atan2(R_PO_I(2,1,ii), R_PO_I(1,1,ii)), 'g'); hold on;
end
legend("Filtered State", "Actual State")
xlabel("displacement $x$ (m)", "interpreter", "latex")
ylabel("displacement $y$ (m)", "interpreter", "latex")
title("movement of robot over time")
grid on;
axis image
hold off;

figure(4);
subplot(3,1,1)
plot(3*sqrt(reshape(P(1,1,:), [n 1])))
xlabel("Time $t$ (s)", "interpreter", "latex")
ylabel("Estimated 3-sigma bounds of $x-\hat{x}$ (m)", "interpreter", "latex")
subplot(3,1,2)
plot(3*sqrt(reshape(P(2,2,:), [n 1])))
xlabel("Time $t$ (s)", "interpreter", "latex")
ylabel("Estimated 3-sigma bounds of $y-\hat{y}$ (m)", "interpreter", "latex")
subplot(3,1,3)
plot(3*sqrt(reshape(P(3,3,:), [n 1])))
xlabel("Time $t$ (s)", "interpreter", "latex")
ylabel("Estimated 3-sigma bounds of $\theta-\hat{\theta}$ (rad)", "interpreter", "latex")

figure(5);
subplot(2,1,1)
plot(z(1,:), 'k+')
xlabel("Time $t$ (s)", "interpreter", "latex")
ylabel("Measurement Error of position component $x$ (m)", "interpreter", "latex")
subplot(2,1,2)
plot(z(2,:), 'k+')
xlabel("Time $t$ (s)", "interpreter", "latex")
ylabel("Measurement Error of position component $x$ (m)", "interpreter", "latex")
