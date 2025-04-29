# Differentia-Drive-AGV
Robot with differential Drive 

%% Simulation Parameters
dt = 0.1;  % Simulation time step (seconds)
t = 0:dt:120;  % Time vector from 0 to 120 seconds
n = length(t);  % Number of simulation steps

%% Robot Parameters
rR = 0.08;  % Right wheel radius (meters)
rL = rR;    % Left wheel radius (meters)
d = 0.5;    % Distance between wheels (meters)



% New Parameters
a = 1;      % Distance from the reference point to the axle (meters)
b = 0.18;   % Distance from center to wheel axis (meters)
Iz = 100;   % Moment of inertia around z-axis
m = 2;      % Mass of the robot (kg)
Ra = 0.1;   % Motor resistance
ka = 0.1;   % Motor constant
kb = 0.1;   % Motor back EMF constant
Be = 0.1;   % Viscous friction coefficient
Ie = 100;   % Electrical inertia
r = 0.08;   % Additional radius parameter
s = 0.1;    % System-specific constant

% Motor Speed Control Parameters
kDT = 40;   
kPT = 20;   
kDR = 60;    
kPR = 30;

%% Derived Constants (Theta Parameters)
theta1 = ((Ra/ka)*(m*r^2 + 2*Ie) + 2*r*kDT) * s * (1/(2*r*kPT));
theta2 = ((Ra/ka)*(Ie*d^2 + 2*r^2*(Iz + m*b^2)) + 2*r*d*kDR) * (1/(2*r*d*kPR)) * s;
theta3 = (Ra/ka) * ((m*b*r)/(2*kPT)) * ((s*m)/(r*a*d^2));
theta4 = (Ra/ka) * (((ka*kb/Ra) + Be) * (1/(r*kPT))) + 1;
theta5 = (Ra/ka) * ((m*b*r)/(d*kPR)) * (s/m);
theta6 = (Ra/ka) * (((ka*kb/Ra) + Be) * (d/(2*r*kPR))) + 1;

%% Desired Trajectory
ud0 = 1;    % Desired linear speed (m/s)
wd0 = 0.1;  % Desired angular speed (rad/s)

%% Initial Conditions
x0 = 0;      % Initial x-position (meters)
y0 = 0;      % Initial y-position (meters)
psi0 = 0;    % Initial orientation angle (radians)
u0 = 0;      % Initial linear velocity (m/s)
w0 = 0;      % Initial angular velocity (rad/s)

%% State Variables Initialization
x = zeros(1, n);      % X-position
y = zeros(1, n);      % Y-position
psi = zeros(1, n);    % Orientation angle
u = zeros(1, n);      % Linear velocity
w = zeros(1, n);      % Angular velocity

% Set initial conditions
x(1) = x0;
y(1) = y0;
psi(1) = psi0;
u(1) = u0;
w(1) = w0;

%% Command Signals (Desired Velocities)
ud = zeros(1, n);
wd = zeros(1, n);

% Split simulation duration into 5 parts
ind1 = find(t <= max(t)/5);
ind2 = find((t > max(t)/5) & (t <= 2*max(t)/5));
ind3 = find((t > 2*max(t)/5) & (t <= 3*max(t)/5));
ind4 = find((t > 3*max(t)/5) & (t <= 4*max(t)/5));
ind5 = find((t > 4*max(t)/5) & (t <= max(t)));

ud(ind1) = 1;  % Move forward
wd(ind1) = 0;  % No rotation

ud(ind2) = 1;  % Continue moving forward
wd(ind2) = 0;  % No rotation

ud(ind3) = 0;  % Stop moving forward
wd(ind3) = 1;  % Turn 90 degrees

ud(ind4) = 1;  % Move forward again
wd(ind4) = 0;  % No rotation

ud(ind5) = 0;  % Stop moving forward
wd(ind5) = 1;  % Turn 90 degrees again


%% Controller Initialization
%% Define a rectangular path
waypoints = [0, 0; 
             10, 0;
             10, 10;
             0, 10;
             0, 0];




%% Simulation Loop (Initial Kinematic Update)
for tk = 1:(n-1)
    % Kinematic Model (eq. (1) in Martins2017)
    AKd = [cos(psi(tk)), -a*sin(psi(tk));
           sin(psi(tk)),  a*cos(psi(tk));
           0,            1];
       
    deltaXd = dt * AKd * [ud(tk); wd(tk)];
    
    x(tk+1) = x(tk) + deltaXd(1);
    y(tk+1) = y(tk) + deltaXd(2);
    psi(tk+1) = psi(tk) + deltaXd(3);
    
    % Placeholder for control loop (to be implemented)
    % u(tk+1) and w(tk+1) will be updated in the control loop below
end




%% PID Controller Gains
KP1 = [0.2, 0; 0, 0.2];  % Proportional gain
KD1 = [0.6, 0; 0, 0.6];  % Derivative gain



%% Initialize Error Variables
ex = zeros(1, n-1);
ey = zeros(1, n-1);
ez = zeros(1, n-1);

% Initialize Time Delay Variables
Ex = zeros(1, ceil(n/10));
Ey = zeros(1, ceil(n/10));

%% Control Loop
for ii = 1:length(waypoints);
    
    % Calculate position error
    ex(ii) = waypoints(ii) - x(ii);
    ey(ii) = waypoints(ii) - y(ii);
    ez(ii) = sqrt((ex(ii)^2 + ey(ii)^2)/2);
    
    % Time delay (simulate sensor delay or processing delay)
    j = ceil(ii/10);   
    if mod(ii, 10) == 1
        Ex(j) = ex(ii);
        Ey(j) = ey(ii);
    end   
    
    % Controller Design
    if j == 1
        s1 = KP1 * [Ex(j); Ey(j)];
    elseif j == 2
        s1 = KD1 * ([Ex(j); Ey(j)] - [Ex(j-1); Ey(j-1)]) + KP1 * [Ex(j); Ey(j)];
    elseif j > 2
        s1 = KD1 * ([Ex(j); Ey(j)] - [Ex(j-1); Ey(j-1)]) + ...
             KP1 * [Ex(j); Ey(j)] + ...
             [waypoints(10*j-9) - 2*waypoints(10*j-19) + waypoints(10*j-29);
              waypoints(10*j-9) - 2*waypoints(10*j-19) + waypoints(10*j-29)];
    end
    
    v1 = s1;    % Control signal (Cruz2008 eq(48))
    
    % Dynamic Model Parameters
    A10 = [cos(psi(ii)),  sin(psi(ii));
           -(1/a)*sin(psi(ii)), (1/a)*cos(psi(ii))];
    A2 = [(theta3/theta1) * (w(ii)^2) - (theta4/theta1) * u(ii);
          -(theta5/theta2) * u(ii) * w(ii) - (theta6/theta2) * w(ii)];
    A1 = [-u(ii)*w(ii)*sin(psi(ii)) - a*(w(ii)^2)*cos(psi(ii));
           u(ii)*w(ii)*cos(psi(ii)) - a*(w(ii)^2)*sin(psi(ii))];
    B1 = [theta1, 0;
          0, theta2];
      
    % Compute Control Input
    uwrt = B1 * (A10 * (v1 - A1) - A2);  % Cruz2008 eq(14)
    
    % Dynamic Model of AGV (eq. (2))
    AK = [cos(psi(ii)), -a*sin(psi(ii));
           sin(psi(ii)),  a*cos(psi(ii));
           0,            1];
    B2 = [1/theta1,  0;
          0,        1/theta2];
    
    vdot = A2 + B2 * uwrt;  % Compute acceleration
    deltauw = dt * vdot;    % Update velocities
    
    % Update Actual Velocities
    u(ii+1) = u(ii) + deltauw(1);
    w(ii+1) = w(ii) + deltauw(2);
    
    % Kinematic Model Update
    deltapose = dt * AK * [u(ii); w(ii)];
    
    % Update Position and Orientation
    x(ii+1) = x(ii) + deltapose(1);
    y(ii+1) = y(ii) + deltapose(2);
    psi(ii+1) = psi(ii) + deltapose(3);
    
    % Optional: Simulate Movement Errors (Ignored)
    % dx(ii) = 0.01*sin(ii*10*dt) + 0.001*sin(4*ii*10*dt);
    % dy(ii) = 0.01*sin(ii*10*dt) + 0.001*sin(4*ii*10*dt);
    % x(ii+1) = x(ii+1) + dx(ii);
    % y(ii+1) = y(ii+1) + dy(ii);
end

%% Visualization and Animation

%% 1. Plot Velocities
figure('Name','Expected and Actual Velocities');
subplot(4,1,1); plot(t, ud, 'b', 'LineWidth', 1.5);
title('Expected Linear Velocity ud');
ylabel('ud (m/s)');
grid on;

subplot(4,1,2); plot(t, u, 'r', 'LineWidth', 1.5);
title('Actual Linear Velocity u');
ylabel('u (m/s)');
grid on;

subplot(4,1,3); plot(t, wd, 'b', 'LineWidth', 1.5);
title('Expected Angular Velocity wd');
ylabel('wd (rad/s)');
grid on;

subplot(4,1,4); plot(t, w, 'r', 'LineWidth', 1.5);
title('Actual Angular Velocity w');
xlabel('Time (s)');
ylabel('w (rad/s)');
grid on;

%% 2. Plot Orientation Angle
figure('Name','Orientation Angle Psi');
subplot(2,1,1); plot(t, psid, 'b', 'LineWidth', 1.5);
title('Desired Orientation Angle \psi_d');
ylabel('\psi_d (rad)');
grid on;

subplot(2,1,2); plot(t, psi, 'r', 'LineWidth', 1.5);
title('Actual Orientation Angle \psi');
xlabel('Time (s)');
ylabel('\psi (rad)');
grid on;

%% 3. Plot Position Errors
figure('Name','Position Errors');
subplot(3,1,1); plot(t(1:end-1), ex, 'm', 'LineWidth', 1.5);
ylabel('ex (m)');
title('Horizontal Error');
grid on;

subplot(3,1,2); plot(t(1:end-1), ey, 'c', 'LineWidth', 1.5);
ylabel('ey (m)');
title('Vertical Error');
grid on;

subplot(3,1,3); plot(t(1:end-1), ez, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('ez (m)');
title('Overall Position Error');
grid on;

%% 4. Workspace Animation with Rectangle Robot and Two Wheels
figure('Name','Workspace Animation');
hold on;
grid on;
axis equal;
xlabel('x (meters)');
ylabel('y (meters)');
title('Workspace with Robot Animation');

% Plot Desired Trajectory
plot(Xd, Yd, 'k-', 'LineWidth', 1.5);

% Plot Start and End Points
plot(waypoints(1), waypoints(1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10); % Start
plot(waypoints(end), waypoints(end), 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 10); % End

% Plot Actual Path (Initial)
hPath = plot(x(1), y(1), 'k--', 'LineWidth', 1.5);

% Define Robot Body Dimensions
L = 0.8;  % Length of the robot body (meters)
W = 0.5;  % Width of the robot body (meters)

% Define Robot Body Corners in Local Frame
local_body = [ L/2,  W/2;
            -L/2,  W/2;
            -L/2, -W/2;
             L/2, -W/2];

% Define Wheels Dimensions
wheel_length = 0.16;  % Length of wheels (meters)
wheel_width = 0.05;  % Width of wheels (meters)

% Define Wheels Positions relative to Body Center
% Adjust the positions as needed bsased on your robot's configuration
left_wheel_pos = [-L/2, W/2 + wheel_width];   % Left Wheel Position
right_wheel_pos = [-L/2, -W/2 - wheel_width]; % Right Wheel Position

% Define Wheels Corners in Local Frame
local_wheel = [ wheel_length/2,  wheel_width/2;
             -wheel_length/2,  wheel_width/2;
             -wheel_length/2, -wheel_width/2;
              wheel_length/2, -wheel_width/2];

% Initial Rotation and Translation for Robot Body
R_initial = [cos(psi(1)), -sin(psi(1));
            sin(psi(1)),  cos(psi(1))];
global_body = (R_initial * local_body')' + [x(1), y(1)];

% Initial Rotation and Translation for Wheels
global_left_wheel = (R_initial * (local_wheel + left_wheel_pos)')' + [x(1), y(1)];
global_right_wheel = (R_initial * (local_wheel + right_wheel_pos)')' + [x(1), y(1)];

% Create Robot Body Patch
hBody = patch('XData', global_body(:,1), 'YData', global_body(:,2), ...
              'FaceColor', 'red', 'EdgeColor', 'black', 'FaceAlpha', 0.7);

% Create Left Wheel Patch
hLeftWheel = patch('XData', global_left_wheel(:,1), 'YData', global_left_wheel(:,2), ...
                   'FaceColor', 'black', 'EdgeColor', 'black');

% Create Right Wheel Patch
hRightWheel = patch('XData', global_right_wheel(:,1), 'YData', global_right_wheel(:,2), ...
                    'FaceColor', 'black', 'EdgeColor', 'black');

% Create Orientation Marker (Front Indicator)
front_marker_length = L/2; % Length of the front indicator
hFront = plot([x(1), x(1) + front_marker_length*cos(psi(1))], ...
              [y(1), y(1) + front_marker_length*sin(psi(1))], ...
              'k-', 'LineWidth', 2);

% Animation Loop: Update Robot Body and Wheels
for ii = 2:n
    % Update Path Plot
    set(hPath, 'XData', x(1:ii), 'YData', y(1:ii));
    
    % Current Orientation
    R = [cos(psi(ii)), -sin(psi(ii));
         sin(psi(ii)),  cos(psi(ii))];
    
    % Rotate and Translate Robot Body
    rotated_body = (R * local_body')';
    global_body = rotated_body + [x(ii), y(ii)];
    set(hBody, 'XData', global_body(:,1), 'YData', global_body(:,2));
    
    % Rotate and Translate Left Wheel
    rotated_left_wheel = (R * (local_wheel + left_wheel_pos)')';
    global_left_wheel = rotated_left_wheel + [x(ii), y(ii)];
    set(hLeftWheel, 'XData', global_left_wheel(:,1), 'YData', global_left_wheel(:,2));
    
    % Rotate and Translate Right Wheel
    rotated_right_wheel = (R * (local_wheel + right_wheel_pos)')';
    global_right_wheel = rotated_right_wheel + [x(ii), y(ii)];
    set(hRightWheel, 'XData', global_right_wheel(:,1), 'YData', global_right_wheel(:,2));
    
    % Update Orientation Marker
    set(hFront, 'XData', [x(ii), x(ii) + front_marker_length*cos(psi(ii))], ...
               'YData', [y(ii), y(ii) + front_marker_length*sin(psi(ii))]);
    
    % Render the updated plot
    drawnow;
    
    % Optional: Pause to control animation speed
    pause(0.01);
end

% Highlight Final Position
plot(x(end), y(end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 10); % End Point

%% Optional: Final Path Overlay
figure('Name','Final Workspace');
hold on;
grid on;
axis equal;
xlabel('x (meters)');
ylabel('y (meters)');
title('Final Workspace with Robot Path');

% Plot Desired Trajectory
plot(waypoints, waypoints, 'k-', 'LineWidth', 1.5);

% Plot Actual Path
plot(x, y, 'r--', 'LineWidth', 1.5);

% Plot Start and End Points
plot(waypoints(1), waypoints(1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10); % Start
plot(waypoints(end), waypoints(end), 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 10); % End
plot(x(end), y(end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 10); % Final Position

legend('Desired Trajectory', 'Actual Path', 'Start Point', 'End Point', 'Final Position');
