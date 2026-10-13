clear;
clc;
close all;

%% 1. SET SYSTEM PARAMETERS
alfa = 1.4;
beta = 1.8;
gamma = 1.05;
m = 13.5;
J = 0.37;

% State Matrix A (6x6)
% x_dot = v_x
% y_dot = v_y
% psi_dot = r
% v_x_dot = -alpha/m * v_x
% v_y_dot = -beta/m * v_y
% r_dot   = -gamma/J * r
A = [0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1;
     0 0 0 -alfa/m 0 0;
     0 0 0 0 -beta/m 0;
     0 0 0 0 0 -gamma/J];

xf = [-1; 2; 3; 0; 0; 0]; % Final Desired State

B = [0 0 0;
     0 0 0;
     0 0 0;
     1/m 0 0;
     0 1/m 0;
     0 0 1/J];
usize = min(size(B));

T = 25;     % Total time
dt = 0.01;  % Time step

% C Matrix: We only measure positions (x, y, psi)
C = zeros(3,6);
C(1:3,1:3) = diag([1,1,1]);

%% 2. CONTROLLABILITY & INPUT CALCULATION (From Lab 2)
syms tau real

tic
disp('Computing Gramian and Control Input... (Please wait)');
eA = expm(A*(T-tau));
dG = eA*B*B'*eA';
G = vpa(int(dG,tau,0,T));

rankG = rank(G);
disp(['rank(G) = ' num2str(rankG)]);

if rankG == 6
    iG = inv(G);
else
    iG = pinv(G);
    warning("System has rank " + rankG + "! It is not fully controllable.")
end

% Minimum Energy Control Formula: u(t) = B' * e^(A'(T-t)) * G^-1 * (xf - e^(AT)x0)
% Note: x0 is zero, so the second term simplifies.
Phi_term = expm(A*T)*zeros(6,1); % Since x0 is zero vector
u_sym = B' * expm(A'*(T-tau)) * iG * (xf - Phi_term);
u = matlabFunction(u_sym, 'Vars', tau); % Convert symbolic to function handle

toc

%% 3. SIMULATION SETUP
time = [0:dt:T]';
LL = length(time);

% Pre-calculate inputs for all time steps to speed up loop
uvalue = zeros(LL, 3);
% Calculate u values (handle vectorization carefully or loop)
% The symbolic function expects scalar time usually, let's map it
for i = 1:LL
    uvalue(i,:) = u(time(i))';
end

x0 = [0 0 0 0 0 0]'; % True initial state (Origin)
x = zeros(6,LL); 
xHat = zeros(6,LL); 
x(:,1) = x0;

%% 4. SET OBSERVER PARAMETERS (Lab 3 Task)
% Design parameters
xi = 1;     % Damping ratio (Critical damping)
wn = 1;     % Natural frequency (rad/s)

% --- Surge Observer Gains (x direction) ---
% Derived from: s^2 + (alpha/m + kx1)s + (kx1*alpha/m + kx4) = s^2 + 2*xi*wn*s + wn^2
kx1 = 2*xi*wn - alfa/m;
kx4 = wn^2 - kx1*(alfa/m);

% --- Sway Observer Gains (y direction) ---
kx2 = 2*xi*wn - beta/m;
kx5 = wn^2 - kx2*(beta/m);

% --- Yaw Observer Gains (psi direction) ---
kx3 = 2*xi*wn - gamma/J;
kx6 = wn^2 - kx3*(gamma/J);

% Construct the Full Observer Gain Matrix Ko (6x3)
Ko = zeros(6,3);
Ko(1:3,1:3) = diag([kx1, kx2, kx3]);
Ko(4:6,1:3) = diag([kx4, kx5, kx6]);

% Initial estimate guess (intentionally wrong as per PDF)
xHat(:,1) = x0 +10000* [1, -1, 1.5, 0.5, -0.5, -1.5]';
y_meas = zeros(3,LL);
yHat = zeros(3,LL);
yHat(:,1) = C * xHat(:,1);

%% 5. SIMULATION LOOP
disp('Simulating...');
for k = 1:LL-1
    
    %% A. System Dynamics (The "Real World")
    u_k = uvalue(k,:)';
    xdot_k = A*x(:,k) + B*u_k;
    x(:,k+1) = x(:,k) + xdot_k * dt; % Euler Integration

    %% B. Measurement (Simulating Sensors)
    y_meas(:,k) = C * x(:,k); % We measure the REAL position

    %% C. Observer (The "Estimator")
    % Observer Dynamics: d(xHat)/dt = A*xHat + B*u + Ko*(y_real - y_estimated)
    y_estimated = C * xHat(:,k);
    dx_est = A*xHat(:,k) + B*u_k + Ko*(y_meas(:,k) - y_estimated);
    
    xHat(:,k+1) = xHat(:,k) + dx_est * dt; % Euler Integration for observer
    
end

%% 6. PLOTS
% --- Plot 1: State Trajectories vs Reference ---
figure('Units', 'normalized', 'OuterPosition', [0 0 1 1]); 
sgtitle("State Evolution (Real vs Reference)") 

titles = {'X Position', 'Y Position', 'Yaw Angle', 'Surge Velocity', 'Sway Velocity', 'Yaw Rate'};
ylabels = {'X [m]', 'Y [m]', 'Yaw [rad]', 'v_x [m/s]', 'v_y [m/s]', 'r [rad/s]'};

for i = 1:6
    subplot(2,3,i);
    plot(time, x(i,:),'linewidth',2); hold on;
    yline(xf(i), 'r--', 'linewidth', 2); % Reference line
    xlabel('Time [s]'); ylabel(ylabels{i}); title(titles{i});
    legend('State','Reference'); grid on;
end

% --- Plot 2: Estimation Error (Real State - Estimated State) ---
figure('Units', 'normalized', 'OuterPosition', [0 0 1 1]); 
sgtitle("State Estimation Error (x - xHat)") 

for i = 1:6
    subplot(2,3,i);
    plot(time, x(i,:) - xHat(i,:),'k','linewidth',2);
    xlabel('Time [s]'); ylabel(['Error ' ylabels{i}]); 
    title(['Error in ' titles{i}]); grid on;
end

% --- Plot 3: Control Inputs ---
figure;
plot(time, uvalue, 'linewidth', 2);
legend('u_1 (Surge Force)','u_2 (Sway Force)','u_3 (Yaw Moment)');
xlabel('Time [s]'); ylabel('Input Magnitude');
title('Control Inputs calculated via Gramian');
grid on;

% --- Plot 4: 2D Trajectory ---
figure;
plot(x(1,:), x(2,:), 'b-', 'linewidth', 2); hold on;
plot(xf(1), xf(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Target
plot(x(1,1), x(2,1), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % Start
xlabel('x [m]'); ylabel('y [m]');
title('2D Ship Trajectory'); legend('Path', 'Target', 'Start');
grid on; axis equal;

% Display Final Errors
disp('------------------------------------------------');
disp(['Final Positioning Error: ', num2str(norm(x(:,end) - xf))]);
disp(['Final Estimation Error Norm: ', num2str(norm(xHat(:,end) - x(:,end)))]);
disp('------------------------------------------------');
