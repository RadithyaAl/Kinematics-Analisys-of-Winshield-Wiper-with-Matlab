% Parameters
Omega0 = 3; % Angular velocity (rad/s)
DLength = 5; % Length of the link
dt = 0.05; % Time step (s)

% Initial conditions
theta = 0; % Initial angular position

% Transformation matrix for rotation
rotation_matrix = @(angle) [cos(angle), -sin(angle), 0;
                            sin(angle),  cos(angle), 0;
                            0,           0,          1];

% Homogeneous coordinates for the link (origin and endpoint)
link_D = [0, DLength; % X-coordinates
          0, 0;       % Y-coordinates
          1, 1];      % Homogeneous coordinates

% Figure setup
figure;
hold on;
axis equal;
grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Rotating Link D (Press Ctrl+C to Stop)');
xlim([-DLength-1, DLength+1]);
ylim([-DLength-1, DLength+1]);

% Continuous rotation
while true
    % Calculate the rotation matrix for the current angle
    R = rotation_matrix(theta);
    
    % Apply the rotation to the link
    transformed_link = R * link_D;
    
    % Extract coordinates
    X = transformed_link(1, :);
    Y = transformed_link(2, :);
    
    % Plot the link as a line
    plot(X, Y, 'b-', 'LineWidth', 2);
    
    % Plot the pivot point
    plot(X(1), Y(1), 'ro', 'MarkerSize', 8, 'DisplayName', 'Pivot'); % Pivot point
    plot(X(2), Y(2), 'go', 'MarkerSize', 6, 'DisplayName', 'Endpoint'); % Endpoint
    
    % Pause for animation effect
    pause(dt);
    
    % Clear the plot for the next frame (except for axis and grid)
    cla;
    
    % Update angular position
    theta = theta + Omega0 * dt; % Increment angle based on angular velocity
end



L2*cos(Theta2) + L3*cos(Theta3) = d + O4B*cos(Theta4)

L2*sin(Theta2) + L3*sin(Theta3) = O4B*sin(Theta4)

last things have to do:
plot the diagram 
a. angular velocity of L2 vs t
b. angular velocity of L3 vs t
c. angular velocity of O4B vs t
d. angular acceleration of L3 vs t
e. angular acceleration of O4B vs t