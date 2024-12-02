% Load sensor data
data = load('filtered_data_for_real.txt'); % Adjust the file name if needed

% Load the additional file with the first 4 columns (time and accelerations)
additional_data = load('LAB1_7.txt'); % Ensure this file has the correct format


% Use the first four columns from the additional data
time = additional_data(:, 1); % Time (s)
acc_x = additional_data(:, 2); % Acceleration in x
acc_y = additional_data(:, 3); % Acceleration in y
acc_z = additional_data(:, 4); % Acceleration in z

% Extract angular velocity data from the original file
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

% Combine the data from both files and the calculated angles
output_data = [time, acc_x, acc_y, acc_z, alpha_vals, beta_vals, gamma_vals];

% Ensure the output matches display precision
disp(output_data(1:10, :)); % Display the first 10 rows for verification

% Save the results to a .txt file in the desired format
output_file = 'formatted_output.txt';
fileID = fopen(output_file, 'w');

% Write the data with higher precision (to avoid incorrect scaling)
for i = 1:size(output_data, 1)
    fprintf(fileID, '%10.1f %8.1f %8.1f %8.1f %12.6f %12.6f %12.6f\n', ...
        output_data(i, 1), output_data(i, 2), output_data(i, 3), ...
        output_data(i, 4), output_data(i, 5), output_data(i, 6), ...
        output_data(i, 7));
end

fclose(fileID);

% Display a message
disp(['Data successfully saved to ', output_file]);


