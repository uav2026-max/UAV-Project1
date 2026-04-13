clear; clc; close all;

%% Parameters
params.m  = 0.027;                 % mass [kg] (Crazyflie 2.1 approx.)
params.g  = 9.81;                  % gravity [m/s^2]

% Inertia matrix [kg.m^2] (approximate values)
params.J  = diag([1.4e-5 1.4e-5 2.17e-5]);

% Geometry
params.l     = 0.0397;             % distance from CoM to rotor [m]
params.alpha = pi/4;               % 45 deg arm geometry

% Rotor drag/thrust ratio for yaw moment
params.kQ_kT = 0.006;              

% Aerodynamic drag coefficients in body axes
params.cDx = 0.0;
params.cDy = 0.0;
params.cDz = 0.0;
% If you want drag effects, try e.g.:
% params.cDx = 2e-4; params.cDy = 2e-4; params.cDz = 3e-4;

%% Input: equal constant thrust on all rotors
T_hover_total = params.m * params.g;
T_hover_each  = T_hover_total / 4;

% "small constant thrust, equal to all rotors"
% Try one of these:
% T_each = 0.95*T_hover_each;   % slightly below hover -> drone descends
% T_each = 1.00*T_hover_each;   % ideal hover
T_each = 1.05*T_hover_each;     % slightly above hover -> drone climbs

u = [T_each; T_each; T_each; T_each];

%% Initial condition: resting state
% x = [px py pz u v w phi theta psi p q r]'
x0 = zeros(12,1);

%% Simulation time
tspan = [0 5];

%% Integrate nonlinear model
[t, x] = ode45(@(t,x) crazyflie_nonlinear_dynamics(t, x, u, params), tspan, x0);

%% Extract states
px    = x(:,1);  py    = x(:,2);  pz    = x(:,3);
ub    = x(:,4);  vb    = x(:,5);  wb    = x(:,6);
phi   = x(:,7);  theta = x(:,8);  psi   = x(:,9);
p     = x(:,10); q     = x(:,11); r     = x(:,12);

%% Plots
figure;
plot(t, px, 'LineWidth', 1.5); hold on;
plot(t, py, 'LineWidth', 1.5);
plot(t, pz, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Position [m]');
legend('p_x','p_y','p_z');
title('Position in inertial frame');

figure;
plot(t, ub, 'LineWidth', 1.5); hold on;
plot(t, vb, 'LineWidth', 1.5);
plot(t, wb, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Body linear velocity [m/s]');
legend('u','v','w');
title('Body-frame linear velocities');

figure;
plot(t, phi*180/pi, 'LineWidth', 1.5); hold on;
plot(t, theta*180/pi, 'LineWidth', 1.5);
plot(t, psi*180/pi, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Euler angles [deg]');
legend('\phi','\theta','\psi');
title('Euler angles');

figure;
plot(t, p, 'LineWidth', 1.5); hold on;
plot(t, q, 'LineWidth', 1.5);
plot(t, r, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
legend('p','q','r');
title('Body angular velocities');

%% Simple 3D trajectory
figure;
plot3(px, py, pz, 'LineWidth', 2);
grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('3D trajectory');
