clc;
clear all;

%% ── Physical Parameters ─────────────────────────────────────────────────
m  = 0.8;    % Mass [kg]
l  = 0.165;  % Arm length [m]
Jx = 0.005;  % Moment of inertia, roll  axis [kg·m²]
Jy = 0.005;  % Moment of inertia, pitch axis [kg·m²]
Jz = 0.009;  % Moment of inertia, yaw   axis [kg·m²]
g  = 9.8067; % Gravitational acceleration [m/s²]

%% ── Aerodynamic Drag Coefficients ───────────────────────────────────────
K1 = 0.01;   % Linear drag coefficient  [N/(m/s)]
K2 = 0;      % Quadratic drag coefficient [N·m/(rad/s)] — set to 0 (ignored)

%% ── Actuator Limits ─────────────────────────────────────────────────────
Tmax = 4;           % Maximum thrust per rotor [N]
Qmax = 0.05;        % Maximum reaction torque per rotor [N·m]
Pmax = 2000;        % Maximum PWM command [μs]
Pmin = Pmax / 2;    % Minimum PWM command [μs]

%% ── PD Position Tracker Gains ───────────────────────────────────────────
% Commanded acceleration: a_c = -Kp*e_p - Kv*e_v - Ka*e_a + a_ref
pos_gain = diag([4.14  4.14  3.105]);  % Kp — position error gain  [1/s²]
vel_gain = diag([2.34  2.34  1.77 ]);  % Kv — velocity error gain  [1/s]
acc_gain = diag([0.5   0.5   0.3  ]);  % Ka — acceleration error gain (dimensionless)

%% ── SMC Attitude Controller Gains ───────────────────────────────────────
% Roll / Pitch axes (shared gains)
% Stability condition: a3 = (2√2 · l · Tmax / Jx) · k6
%                      k5 > 0
a3 = 32.764;   % Sliding surface pole [rad/s]
k5 = 0.0388;   % SMC switching gain (roll/pitch)
k6 = 0.0219;   % SMC damping gain   (roll/pitch)

% Yaw axis
% Stability condition: a4 = (4 · Qmax / Jz) · k8
%                      k7 > 0
a4 = 13.82;    % Sliding surface pole [rad/s]
k7 = 0.3147;   % SMC switching gain (yaw)
k8 = 0.1244;   % SMC damping gain   (yaw)