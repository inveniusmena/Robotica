% Load sensor data
data = load('filtered_data.txt'); % Adjust the file name if needed

% Extract time and rate-gyro data
time = data(:, 1) * 1e-6; % Convert time from microseconds to seconds
w_x = data(:, 5); % Angular velocity along x-axis
w_y = data(:, 6); % Angular velocity along y-axis
w_z = data(:, 7); % Angular velocity along z-axis
a_x = data(:, 2); % Accelerometer x-axis
a_y = data(:, 3); % Accelerometer y-axis
a_z = data(:, 4); % Accelerometer z-axis

% Step 1: Compute initial orientation using the accelerometer (assuming no initial rotation)
% Normalize accelerometer data to find initial pitch and roll
acc_mag = sqrt(a_x(1)^2 + a_y(1)^2 + a_z(1)^2);
ax = a_x(1) / acc_mag;
ay = a_y(1) / acc_mag;
az = a_z(1) / acc_mag;

% Initial roll (alpha) and pitch (beta) from accelerometer
alpha = atan2(ay, az);  % Roll
beta = atan2(-ax, sqrt(ay^2 + az^2)); % Pitch
gamma = 0; % Assume initial yaw (gamma) is 0

% Step 2: Initialize arrays to store Euler angles over time
alpha_vals = zeros(size(time));
beta_vals = zeros(size(time));
gamma_vals = zeros(size(time));

% Store initial values
alpha_vals(1) = alpha;
beta_vals(1) = beta;
gamma_vals(1) = gamma;

% Step 3: Runge-Kutta integration loop for computing Euler angles
for i = 1:length(time)-1
    % Time step
    dt = time(i+1) - time(i);

    % Rotation matrix R (Euler Angles: Roll-Pitch-Yaw)
    R = [
        1, sin(alpha)*tan(beta), cos(alpha)*tan(beta); 
        0, cos(alpha), -sin(alpha);
        0, sin(alpha)/cos(beta), cos(alpha)/cos(beta)
    ];

    % Angular velocity vector
    omega = [w_x(i); w_y(i); w_z(i)];

    % Compute angular velocity in Euler angle space (derivatives)
    omega_euler = R \ omega; % Solve for angular velocity in Euler angle space
    alpha_dot = omega_euler(1);
    beta_dot = omega_euler(2);
    gamma_dot = omega_euler(3);

    % Step 4: Runge-Kutta integration (RK4 method)
    
    % First estimate (k1)
    k1_alpha = alpha_dot;
    k1_beta = beta_dot;
    k1_gamma = gamma_dot;

    % Second estimate (k2)
    k2_alpha = alpha_dot + 0.5 * k1_alpha * dt;
    k2_beta = beta_dot + 0.5 * k1_beta * dt;
    k2_gamma = gamma_dot + 0.5 * k1_gamma * dt;

    % Third estimate (k3)
    k3_alpha = alpha_dot + 0.5 * k2_alpha * dt;
    k3_beta = beta_dot + 0.5 * k2_beta * dt;
    k3_gamma = gamma_dot + 0.5 * k2_gamma * dt;

    % Fourth estimate (k4)
    k4_alpha = alpha_dot + k3_alpha * dt;
    k4_beta = beta_dot + k3_beta * dt;
    k4_gamma = gamma_dot + k3_gamma * dt;

    % Compute final updates (weighted average)
    alpha = alpha + (dt / 6) * (k1_alpha + 2 * k2_alpha + 2 * k3_alpha + k4_alpha);
    beta = beta + (dt / 6) * (k1_beta + 2 * k2_beta + 2 * k3_beta + k4_beta);
    gamma = gamma + (dt / 6) * (k1_gamma + 2 * k2_gamma + 2 * k3_gamma + k4_gamma);

    % Store the Euler angles at each time step
    alpha_vals(i+1) = alpha;
    beta_vals(i+1) = beta;
    gamma_vals(i+1) = gamma;
end

% Step 5: Plot Euler angles over time
figure;
plot(time, alpha_vals, 'r', 'LineWidth', 1.5); hold on;
plot(time, beta_vals, 'g', 'LineWidth', 1.5);
plot(time, gamma_vals, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angle (radians)');
title('Orientation Trajectory (Euler Angles) with Runge-Kutta');
legend('\alpha (Roll)', '\beta (Pitch)', '\gamma (Yaw)');
grid on;
hold off;
