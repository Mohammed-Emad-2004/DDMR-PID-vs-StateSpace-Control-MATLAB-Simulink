% --- Physical Robot & Track Parameters ---
% Scale the track down to realistically fit a 0.2m wide robot
track_pts = [2*cos(linspace(0, 2*pi, 1000)'), 2*sin(linspace(0, 2*pi, 1000)').*cos(linspace(0, 2*pi, 1000)')];
L_f = 0.2;         % Sensor distance forward from robot center
W_spread = 0.15;   % Total width of the 6-sensor array
line_width = 0.03; % Detection threshold radius
R = 0.05;          % Physical wheel radius (PROTECTED: Do not overwrite with LQR R matrix)
L = 0.2;           % Physical wheelbase (PROTECTED: Do not overwrite with Observer L matrix)

assignin('base', 'track_pts', track_pts);
assignin('base', 'L_f', L_f);
assignin('base', 'W_spread', W_spread);
assignin('base', 'line_width', line_width);
assignin('base', 'R', R);
assignin('base', 'L', L);

% --- 1. Plant Parameters ---
V_ref = 0.3; % Base cruising speed (m/s)

% --- 2. Full System Matrices (3 States, 2 Inputs) ---
% States: x = [|e_y|_brake; e_y; e_theta]
% Inputs: u = [v_brake; omega]
A_full = [0,  0,     0; 
          0,  0, -V_ref; 
          0,  0,     0];
B_full = [1, 0; 
          0, 0; 
          0, 1];
C_full = [0, 1, 0];

% Steering Subsystem (for Observer)
A_steer = [0, -V_ref; 0, 0];
B_steer = [0; 1];
C_steer = [1, 0];

% --- 3. Controllability & Observability Proofs ---
Co = ctrb(A_full, B_full);
disp(['Full System Controllability Rank (Expected: 3): ', num2str(rank(Co))]);
Ob_steer = obsv(A_steer, C_steer);
disp(['Steering Observability Rank (Expected: 2): ', num2str(rank(Ob_steer))]);

% --- 4. Winning LQR Controller Gains (K) for H-Bridge & Motor ---
q_brake  = 2.25;   % K_brake = 1.50 (Preserves high cruising speed through corners!)
q_ey     = 4500;   % K_steer(1) = -67.08 (Crisp lateral authority matching [-4, -5] scale)
q_etheta = 65;     % K_steer(2) = 10.26 (Ideal damping ratio zeta = 1.14 without sluggishness)

Q_lqr = diag([q_brake, q_ey, q_etheta]);
R_lqr = diag([1.0, 1.0]);

K = lqr(A_full, B_full, Q_lqr, R_lqr);

% --- 5. Winning Observer Gain (L_obs) for H-Bridge & Motor ---
Q_obs = diag([25, 250000]); 
R_obs = 1.0;              
L_obs = lqr(A_steer', C_steer', Q_obs, R_obs)';
% Produces L_obs = [17.46; -500.0] (Fast tracking that beats H-Bridge lag)
% --- 6. Send to Workspace ---
assignin('base', 'K', K);
assignin('base', 'L_obs', L_obs);
assignin('base', 'A_steer', A_steer);
assignin('base', 'B_steer', B_steer);
assignin('base', 'C_steer', C_steer);

disp('--- Final LQR K Matrix ---'); disp(K);
disp('--- Final Observer Matrix L_obs ---'); disp(L_obs);

% --- 7. Motor & PID Parameters ---
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