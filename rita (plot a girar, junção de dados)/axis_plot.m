% Load sensor data
data = load('filtered_data_for_real.txt'); % Adjust the file name if needed

additional_data = load('filtered_data_outliers.txt'); % Ensure this file has the correct format

acc_x = additional_data(:, 1); % Acceleration in x
acc_y = additional_data(:, 2); % Acceleration in y
acc_z = additional_data(:, 3); % Acceleration in z


% Extract time and rate-gyro data
time = data(:, 1); % Convert time from microseconds to seconds
w_x = data(:, 5) * pi / 180; % Convert angular velocity along x-axis to radians/s
w_y = data(:, 6) * pi / 180; % Convert angular velocity along y-axis to radians/s
w_z = data(:, 7) * pi / 180; % Convert angular velocity along z-axis to radians/s

% Initial basis vectors
x_axis = [1; 0; 0];
y_axis = [0; 1; 0];
z_axis = [0; 0; 1];


% Step 1: Initialize Euler angles (roll, pitch, yaw) to zero
alpha = 0; % Roll (initial orientation)
beta = 0;  % Pitch (initial orientation)
gamma = 0; % Yaw (initial orientation)

% Step 2: Initialize arrays to store Euler angles over time
alpha_vals = zeros(size(time));
beta_vals = zeros(size(time));
gamma_vals = zeros(size(time));

% Step 3: Loop through the data and integrate angular velocities to get Euler angles
for i = 1:length(time)-1
    % Time difference
    dt = time(i+1) - time(i);

    % Rotation matrix R (Euler Angles: Roll-Pitch-Yaw)
    R = [
        1, sin(alpha)*tan(beta), cos(alpha)*tan(beta); 
        0, cos(alpha), -sin(alpha);
        0, sin(alpha)/cos(beta), cos(alpha)/cos(beta)
    ];

    % Angular velocity vector
    omega = [w_x(i); w_y(i); w_z(i)];

    % Compute angular velocity in the Euler angle space (derivatives)
    omega_euler = R \ omega; % Solve for angular velocity in Euler angle space

    % Extract the time derivatives (angular velocities)
    alpha_dot = omega_euler(1);
    beta_dot = omega_euler(2);
    gamma_dot = omega_euler(3);

    % Integrate to get Euler angles (using simple Euler integration)
    alpha = alpha + alpha_dot * dt;
    beta = beta + beta_dot * dt;
    gamma = gamma + gamma_dot * dt;

    % Store the Euler angles at each time step
    alpha_vals(i+1) = alpha;
    beta_vals(i+1) = beta;
    gamma_vals(i+1) = gamma;
end

disp(alpha_vals);

%% Plots

% Plot the angular velocities
figure;
plot(time, w_x, 'r', 'LineWidth', 1.5); hold on;
plot(time, w_y, 'g', 'LineWidth', 1.5);
plot(time, w_z, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocity Components (\omega_x, \omega_y, \omega_z)');
legend('\omega_x', '\omega_y', '\omega_z');
grid on;
hold off;

% Figure setup
figure; 
axis equal; grid on;
hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-1.5, 1.5]); ylim([-1.5, 1.5]); zlim([-1.5, 1.5]);
title('3D Rotating Coordinate Axes');
view(3); % Set a 3D view angle

% Plot the initial axes to ensure the plot doesn't clear each time
h_x = quiver3(0, 0, 0, 1, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
h_y = quiver3(0, 0, 0, 0, 1, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
h_z = quiver3(0, 0, 0, 0, 0, 1, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Text handles for labels
tx = text(1, 0, 0, 'X', 'Color', 'r', 'FontSize', 10, 'HorizontalAlignment', 'left');
ty = text(0, 1, 0, 'Y', 'Color', 'g', 'FontSize', 10, 'HorizontalAlignment', 'left');
tz = text(0, 0, 1, 'Z', 'Color', 'b', 'FontSize', 10, 'HorizontalAlignment', 'left');

% Loop through the data
for i = 1:length(time)
    % Rotation matrices for roll, pitch, yaw
    R_roll = rotx(rad2deg(alpha_vals(i)));  % Roll rotation matrix
    R_pitch = roty(rad2deg(beta_vals(i))); % Pitch rotation matrix
    R_yaw = rotz(rad2deg(gamma_vals(i)));    % Yaw rotation matrix

    % Combined Roll-Pitch-Yaw rotation
    R = R_yaw * R_pitch * R_roll; 

    % Rotated axes
    new_x = R * x_axis;
    new_y = R * y_axis;
    new_z = R * z_axis;

    % Update the quivers (axes)
    set(h_x, 'UData', new_x(1), 'VData', new_x(2), 'WData', new_x(3));
    set(h_y, 'UData', new_y(1), 'VData', new_y(2), 'WData', new_y(3));
    set(h_z, 'UData', new_z(1), 'VData', new_z(2), 'WData', new_z(3));

    % Update floating labels
    set(tx, 'Position', new_x, 'String', 'X');
    set(ty, 'Position', new_y, 'String', 'Y');
    set(tz, 'Position', new_z, 'String', 'Z');

    % Fix the observation window
    xlim([-1.5, 1.5]); ylim([-1.5, 1.5]); zlim([-1.5, 1.5]);
    view(3); % Ensure the 3D view remains fixed

    pause(0.1); % Visualization pause
end


% Step 4: Plot Euler angles over time
figure;
plot(time, alpha_vals, 'r', 'LineWidth', 1.5); hold on;
plot(time, beta_vals, 'g', 'LineWidth', 1.5);
plot(time, gamma_vals, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angle (radians)');
title('Orientation Trajectory (Euler Angles)');
legend('\alpha (Roll)', '\beta (Pitch)', '\gamma (Yaw)');
grid on;
hold off;

disp(['Size of time: ', mat2str(size(time))]);
disp(['Size of acc_x: ', mat2str(size(acc_x))]);
disp(['Size of acc_y: ', mat2str(size(acc_y))]);
disp(['Size of acc_z: ', mat2str(size(acc_z))]);
disp(['Size of alpha_vals: ', mat2str(size(alpha_vals))]);
disp(['Size of beta_vals: ', mat2str(size(beta_vals))]);
disp(['Size of gamma_vals: ', mat2str(size(gamma_vals))]);

%% Joining of Files

% % Combine the data from both files and the calculated angles
% output_data = [time, acc_x, acc_y, acc_z, alpha_vals, beta_vals, gamma_vals];




% % Ensure the output matches display precision
% disp(output_data(1:10, :)); % Display the first 10 rows for verification
% 
% % Save the results to a .txt file in the desired format
% output_file = 'formatted_output.txt';
% fileID = fopen(output_file, 'w');
% 
% % Write the data with higher precision (to avoid incorrect scaling)
% for i = 1:size(output_data, 1)
%     fprintf(fileID, '%10.1f %8.1f %8.1f %8.1f %12.6f %12.6f %12.6f\n', ...
%         output_data(i, 1), output_data(i, 2), output_data(i, 3), ...
%         output_data(i, 4), output_data(i, 5), output_data(i, 6), ...
%         output_data(i, 7));
% end
% 
% fclose(fileID);
% 
% % Display a message
% disp(['Data successfully saved to ', output_file]);

