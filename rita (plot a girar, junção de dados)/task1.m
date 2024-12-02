data = load('LAB1_7.txt');

time_ms = data(:, 1);       % Extract time column in milliseconds
time_s = time_ms / 1000000;    % Convert milliseconds to seconds
accel = data(:, 2:4);         % Columns 2-4: Accelerometer
gyro = data(:, 5:7);          % Columns 5-7: Gyroscope

figure;

% Plot accelerometer data
subplot(2, 1, 1); % Create subplot for accelerometer
plot(time_s, accel(:, 1), 'r', 'DisplayName', 'X');
hold on;
plot(time_s, accel(:, 2), 'g', 'DisplayName', 'Y');
plot(time_s, accel(:, 3), 'b', 'DisplayName', 'Z'); %Probably Z-axis because it oscillates between -g and g (g is gravity)
hold off;
title('Accelerometer Data');
xlabel('Time (s)');
ylabel('Acceleration (mG)');
legend; % Add legend to differentiate axes

% Plot gyroscope data
subplot(2, 1, 2); % Create subplot for gyroscope
plot(time_s, gyro(:, 1), 'r', 'DisplayName', 'Roll');
hold on;
plot(time_s, gyro(:, 2), 'g', 'DisplayName', 'Pitch');
plot(time_s, gyro(:, 3), 'b', 'DisplayName', 'Yaw');
hold off;
title('Gyroscope Data');
xlabel('Time (s)');
ylabel('Angular Velocity (degree/second)');
legend; % Add legend to differentiate axes
