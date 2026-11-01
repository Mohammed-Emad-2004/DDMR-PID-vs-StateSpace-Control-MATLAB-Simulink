% --- Physical Robot & Track Parameters ---
% Scale the track down to realistically fit a 0.2m wide robot
track_pts = [2*cos(linspace(0, 2*pi, 1000)'), 2*sin(linspace(0, 2*pi, 1000)').*cos(linspace(0, 2*pi, 1000)')];
L_f = 0.2;         % Sensor distance forward from robot center
W_spread = 0.15;   % Total width of the 6-sensor array
line_width = 0.03; % Detection threshold radius
R = 0.05;          % Physical wheel radius
L = 0.2;           % Physical wheelbase (SAFE: Used by BOTH SS and PID kinematics)

assignin('base', 'track_pts', track_pts);
assignin('base', 'L_f', L_f);
assignin('base', 'W_spread', W_spread);
assignin('base', 'line_width', line_width);
assignin('base', 'R', R);
assignin('base', 'L', L);

% --- 1. Plant Parameters ---
V_ref = 0.3 ; % Base cruising speed (m/s)

% --- 2. Full System Matrices (3 States, 2 Inputs) ---
A_full = [0,  0,    0; 
          0,  0, -V_ref; 
          0,  0,    0];
B_full = [1, 0; 
          0, 0; 
          0, 1];
C_full = [0, 1, 0];

% --- 3. Controllability & Observability Proofs ---
Co = ctrb(A_full, B_full);
disp(['Full System Controllability Rank (Expected: 3): ', num2str(rank(Co))]);

A_steer = [0, -V_ref; 0, 0];
B_steer = [0; 1];
C_steer = [1, 0];
Ob_steer = obsv(A_steer, C_steer);
disp(['Steering Observability Rank (Expected: 2): ', num2str(rank(Ob_steer))]);

% --- 4. Controller Gains (K) ---
% Braking Pole (Relaxed for smoother corner entry)
pole_brake = -1.5;
K_brake = place(0, 1, pole_brake);

% Steering Poles (Critically damped to eliminate oscillations)
%poles_steer = [-.9, -1.89];
poles_steer = [-4, -5];

K_steer = place(A_steer, B_steer, poles_steer);

% Unified 2x3 Matrix
K = [K_brake, 0, 0; 
     0, K_steer(1), K_steer(2)];

% --- 5. Observer Gain (L_obs) ---
% Observer poles (Fast enough to track, slow enough to filter sensor noise)
poles_obs = [-10, -15];
L_obs = place(A_steer', C_steer', poles_obs)'; % RENAMED TO AVOID OVERWRITING 'L'

% --- 6. Send to Workspace ---
assignin('base', 'K', K);
assignin('base', 'L_obs', L_obs); % Exported with the new name
assignin('base', 'A_steer', A_steer);
assignin('base', 'B_steer', B_steer);
assignin('base', 'C_steer', C_steer);

disp('--- Final K Matrix ---'); disp(K);
disp('--- Final Observer Matrix L_obs ---'); disp(L_obs);

motor_V_nom = 6;
motor_R_arm = 2;
motor_w_nl = 105;
motor_I_nl = 0.04;
motor_T_stall = 0.08;

ph = 0.167;
ih = 0.0001;
dh = 0.009;

ps = 0.05;
is = 0;
Ds = 0.01;