% Parameters
L2 = 186; % Length of Link 2
L3 = 2.9 * L2; % Length of Link 3
O4B = 1.2 * L2; % Distance O4 to B
O4C_length = 3.5 * L2; % Length of Link O4C
d = 3 * L2; % Distance between pivots
Omega2 = 6; % Angular velocity of Link 2 (rad/s)
Theta2 = 0; % Initial angle of Link 2
dt = 0.01; % Time step
max_time = 10; % Maximum simulation time

% Initialize variables
Theta3_past = 0;
Omega3_past = 0;

% Options for fsolve
options = optimoptions('fsolve', 'Display', 'off');

% Initial guesses for Theta3 and Theta4
Theta3_guess = pi; % Initial guess for Theta3
Theta4_guess = pi; % Initial guess for Theta4

% Initial conditions
t = 0; % Start time
Initial_Coordinate_L2 = [0; 0];

% Figure setup for mechanism plotting
figure(1);
hold on;
grid on;
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
title('Continuous Kinematics of the Mechanism');
xlim([-4 * L2, 8 * L2]);
ylim([-4 * L2, 6 * L2]);

% Create animated lines for velocity and acceleration
figure(2);
Omega3_plot = animatedline('Color', 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocity of Link L3');
grid on;

figure(3);
alpha3_plot = animatedline('Color', 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular Acceleration (rad/s^2)');
title('Angular Acceleration of Link L3');
grid on;

% Simulation loop
while t < max_time
    % Update time and Theta2
    t = t + dt;
    Theta2 = (pi / 2) + Omega2 * t; % Angular position of Link 2
    
    % Solve for Theta3 and Theta4
    eqns = @(angles) [
    L3 * cos(angles(1)) - (d + O4B * cos(asin((L2 * sin(Theta2) + L3 * sin(angles(1))) / O4B)) - L2 * cos(Theta2));
    O4B * sin(angles(2)) - (L2 * sin(Theta2) + L3 * sin(acos((d + O4B * cos(angles(2)) - L2 * cos(Theta2)) / L3)))
];

    solution = fsolve(eqns, [Theta3_guess, Theta4_guess], options);
    Theta3 = solution(1);
    Theta4 = solution(2);
    Theta3_current = Theta3;
    
    % Calculate positions of key points
    R1 = Initial_Coordinate_L2 + [L2 * cos(Theta2);
                                  L2 * sin(Theta2)];
    R3 = R1 + [L3 * cos(Theta3);
               L3 * sin(Theta3)];
    R4 = [d; 0] + [O4B * cos(Theta4);
                   O4B * sin(Theta4)];
    R5 = [d; 0] + [O4C_length * cos(Theta4 + pi);
                   O4C_length * sin(Theta4 + pi)];
    
    % Calculate angular velocity
    Omega3_current = (Theta3_current - Theta3_past) / dt;
    
    % Calculate angular acceleration
    alpha3_current = (Omega3_current - Omega3_past) / dt;
    
    % Update past values
    Theta3_past = Theta3_current;
    Omega3_past = Omega3_current;
    
    % Update animated lines for angular velocity and acceleration
    addpoints(Omega3_plot, t, Omega3_current);
    % addpoints(alpha3_plot, t, alpha3_current);
    
    % Plot mechanism links
    figure(1);
    cla; % Clear current axes
    plot([Initial_Coordinate_L2(1), R1(1), R3(1)], ...
         [Initial_Coordinate_L2(2), R1(2), R3(2)], 'k-o', 'LineWidth', 2);
    hold on;
    plot([R3(1), R4(1), R5(1)], ...
         [R3(2), R4(2), R5(2)], 'r-o', 'LineWidth', 2);
    
    % Update plots
    drawnow;
end
