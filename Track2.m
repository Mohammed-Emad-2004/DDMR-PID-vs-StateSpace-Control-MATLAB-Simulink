%% --- Physical Robot & Track Parameters ---
L_f = 0.2;           % Sensor distance forward from robot center (m)
W_spread = 0.15;     % Total width of the 6-sensor array (m)
line_width = 0.03;   % Detection threshold radius (m) (Tape width = 0.06m)
R = 0.05;            % Physical wheel radius (PROTECTED: Do not overwrite with LQR R matrix)
L = 0.2;             % Physical wheelbase (PROTECTED: Do not overwrite with Observer L matrix)

% Track Size Multiplier: 1.0 = Large 12.3m x 9.6m course (~2.8x longer than before).
% Change to 0.8 or 1.3 anytime to fine-tune total track length while keeping line_width = 0.03m!
length_scale = 1.0;  
theta_start  = -pi/4; % Matches your Simulink model's initial heading at (0,0)

[track_pts, segments_cell] = generate_challenging_track(theta_start, line_width, length_scale);

assignin('base', 'track_pts', track_pts);
assignin('base', 'L_f', L_f);
assignin('base', 'W_spread', W_spread);
assignin('base', 'line_width', line_width);
assignin('base', 'R', R);
assignin('base', 'L', L);

%% --- 1. Plant Parameters ---
V_ref = 0.3; % Base cruising speed (m/s)

%% --- 2. Full System Matrices (3 States, 2 Inputs) ---
% States: x = [|e_y|_brake; e_y; e_theta]
% Inputs: u = [v_brake; omega]
A_full = [0,  0,      0; 
          0,  0, -V_ref; 
          0,  0,      0];
B_full = [1, 0; 
          0, 0; 
          0, 1];
C_full = [0, 1, 0];

% Steering Subsystem (for Observer)
A_steer = [0, -V_ref; 0, 0];
B_steer = [0; 1];
C_steer = [1, 0];

%% --- 3. Controllability & Observability Proofs ---
Co = ctrb(A_full, B_full);
disp(['Full System Controllability Rank (Expected: 3): ', num2str(rank(Co))]);
Ob_steer = obsv(A_steer, C_steer);
disp(['Steering Observability Rank (Expected: 2): ', num2str(rank(Ob_steer))]);

%% --- 4. Winning LQR Controller Gains (K) for H-Bridge & Motor ---
q_brake  = 2.25;   % K_brake = 1.50
q_ey     = 4500;   % K_steer(1) = -67.08
q_etheta = 65;     % K_steer(2) = 10.26

Q_lqr = diag([q_brake, q_ey, q_etheta]);
R_lqr = diag([1.0, 1.0]);

K = lqr(A_full, B_full, Q_lqr, R_lqr);

%% --- 5. Winning Observer Gain (L_obs) for H-Bridge & Motor ---
Q_obs = diag([25, 250000]); 
R_obs = 1.0;               
L_obs = lqr(A_steer', C_steer', Q_obs, R_obs)';

%% --- 6. Send to Workspace ---
assignin('base', 'K', K);
assignin('base', 'L_obs', L_obs);
assignin('base', 'A_steer', A_steer);
assignin('base', 'B_steer', B_steer);
assignin('base', 'C_steer', C_steer);

disp('--- Final LQR K Matrix ---'); disp(K);
disp('--- Final Observer Matrix L_obs ---'); disp(L_obs);

%% --- 7. Motor & PID Parameters ---
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

%% ========================================================================
%  HELPER FUNCTION: ENLARGED 8-FEATURE TRACK & SIMSCAPE STL GENERATOR
%  ========================================================================
function [track_pts, segments] = generate_challenging_track(theta_rot, half_width, S)
    ds = 0.005; % 5 mm point spacing for accurate sensor lookup
    segments = {};
    
    % --- FEATURE 1: Launch Straight (0.80m) + 5 Long Dotted Line Dashes ---
    u_s1 = (0 : ds : 0.80*S)';
    segments{end+1} = [u_s1, zeros(size(u_s1))];
    
    gap_len  = 0.14 * S; % 14 cm blank gap
    dash_len = 0.36 * S; % 36 cm solid dash
    u_curr   = 0.80*S + gap_len;
    for k = 1:5
        u_dash = (u_curr : ds : u_curr + dash_len)';
        segments{end+1} = [u_dash, zeros(size(u_dash))];
        u_curr = u_curr + dash_len + gap_len;
    end
    % After 5 dashes + gap, u_curr = 3.44 * S
    
    % --- FEATURE 2: Horizontal Pass Through 90-Deg Intersection at (4.30*S, 0) ---
    u_int1 = (u_curr : ds : 5.20*S)';
    cont_pts = [u_int1, zeros(size(u_int1))];
    
    % --- FEATURE 3: Spacious Sawtooth Zig-Zag (Each diagonal ramp > 1.0m long) ---
    zz_wp = S * [5.20,  0.00;
                 5.65,  0.32;
                 6.45, -0.32;
                 7.25,  0.32;
                 8.05, -0.32;
                 8.50,  0.00;
                 9.30,  0.00]; % 0.80m straight before sweeping curve
    for i = 1:(size(zz_wp,1)-1)
        p1 = zz_wp(i,:); p2 = zz_wp(i+1,:);
        n_steps = max(2, round(norm(p2 - p1)/ds));
        u_seg = linspace(p1(1), p2(1), n_steps)';
        v_seg = linspace(p1(2), p2(2), n_steps)';
        cont_pts = [cont_pts; [u_seg(2:end), v_seg(2:end)]]; %#ok<AGROW>
    end
    
    % --- FEATURE 4: Large 90-Degree Sweeping Curve (R = 1.50m) ---
    % Center at (9.30*S, 1.50*S), turns from -pi/2 to 0 -> ends at (10.80*S, 1.50*S)
    R_sweep = 1.50 * S;
    th_sw = linspace(-pi/2, 0, round(R_sweep*(pi/2)/ds))';
    u_sw = 9.30*S + R_sweep*cos(th_sw);
    v_sw = 1.50*S + R_sweep*sin(th_sw);
    cont_pts = [cont_pts; [u_sw(2:end), v_sw(2:end)]];
    
    % --- FEATURE 5: Long C2-Smooth S-Bend Chicane (Length = 3.60m, Amp = 0.55m) ---
    L_ch = 3.60 * S;
    v_ch = (1.50*S : ds : 1.50*S + L_ch)';
    tau = (v_ch - 1.50*S) / L_ch;
    u_ch = 10.80*S + (0.55*S / 1.299) * (sin(2*pi*tau) - 0.5*sin(4*pi*tau));
    cont_pts = [cont_pts; [u_ch(2:end), v_ch(2:end)]];
    
    % Straight into hairpin (v = 5.10*S to 5.70*S at u = 10.80*S)
    v_pre_hp = (5.10*S : ds : 5.70*S)';
    cont_pts = [cont_pts; [10.80*S*ones(size(v_pre_hp(2:end))), v_pre_hp(2:end)]];
    
    % --- FEATURE 6: 180-Degree Hairpin Turn (R = 0.95m) ---
    % Center at (9.85*S, 5.70*S), turns from 0 to pi -> ends at (8.90*S, 5.70*S)
    R_hp = 0.95 * S;
    th_hp = linspace(0, pi, round(R_hp*pi/ds))';
    u_hp = 9.85*S + R_hp*cos(th_hp);
    v_hp = 5.70*S + R_hp*sin(th_hp);
    cont_pts = [cont_pts; [u_hp(2:end), v_hp(2:end)]];
    
    % --- FEATURE 7: Sharp 90-Deg Corner + Dual Sharp 45-Deg Turns + Intersection ---
    sharp_wp = S * [8.90,  5.70;
                    8.90,  4.10;  % <--- Sharp 90-degree corner vertex (after 1.60m straight)
                    6.50,  4.10;  % <--- Sharp 45-degree corner #1 (after 2.40m straight)
                    4.30,  1.90;  % <--- Sharp 45-degree corner #2 (after 3.11m diagonal)
                    4.30, -1.80]; % <--- Crosses Intersection at (4.30*S, 0) vertically
    for i = 1:(size(sharp_wp,1)-1)
        p1 = sharp_wp(i,:); p2 = sharp_wp(i+1,:);
        n_steps = max(2, round(norm(p2 - p1)/ds));
        u_seg = linspace(p1(1), p2(1), n_steps)';
        v_seg = linspace(p1(2), p2(2), n_steps)';
        cont_pts = [cont_pts; [u_seg(2:end), v_seg(2:end)]]; %#ok<AGROW>
    end
    
    % --- FEATURE 8: Spacious Return Loop Closing Back to Origin (0,0) ---
    % 90-deg curve centered at (3.10*S, -1.80*S) with R = 1.20*S -> reaches (3.10*S, -3.00*S)
    R_r1 = 1.20 * S;
    th_r1 = linspace(0, -pi/2, round(R_r1*(pi/2)/ds))';
    u_r1 = 3.10*S + R_r1*cos(th_r1);
    v_r1 = -1.80*S + R_r1*sin(th_r1);
    cont_pts = [cont_pts; [u_r1(2:end), v_r1(2:end)]];
    
    % Long 3.10m straight along v = -3.00*S from u = 3.10*S to u = 0
    u_r2 = (3.10*S : -ds : 0.00)';
    cont_pts = [cont_pts; [u_r2(2:end), -3.00*S*ones(size(u_r2(2:end)))]];
    
    % 180-deg return curve centered at (0, -1.50*S) with R = 1.50*S -> lands at (0,0) heading +u
    R_r2 = 1.50 * S;
    th_r2 = linspace(-pi/2, -3*pi/2, round(R_r2*pi/ds))';
    u_r3 = 0.00 + R_r2*cos(th_r2);
    v_r3 = -1.50*S + R_r2*sin(th_r2);
    cont_pts = [cont_pts; [u_r3(2:end), v_r3(2:end)]];
    
    segments{end+1} = cont_pts;
    
    % --- Rotate All Segments by theta_rot (-pi/4) to Match Simulink Start Pose ---
    R_mat = [cos(theta_rot), -sin(theta_rot); sin(theta_rot), cos(theta_rot)];
    track_pts = [];
    for i = 1:length(segments)
        segments{i} = (R_mat * segments{i}')';
        track_pts = [track_pts; segments{i}]; %#ok<AGROW>
    end
    
    % --- Export 3D CAD Mesh (.STL) for Simscape File Solid Block ---
    stl_filename = 'challenging_track.stl';
    export_track_to_stl(segments, half_width, 0.002, stl_filename);
    
    % --- Plot Labeled Verification Figure (with 0.4m Robot Chassis Circle for Scale) ---
    figure('Name', 'Enlarged 8-Feature DDMR Proving Ground', 'Color', 'w');
    hold on; grid on; axis equal;
    for i = 1:length(segments)
        plot(segments{i}(:,1), segments{i}(:,2), 'k-', 'LineWidth', 2.2);
    end
    % Draw a 0.4m diameter blue circle at (0,0) representing the physical robot size
    th_circ = linspace(0, 2*pi, 60);
    fill(0.20*cos(th_circ), 0.20*sin(th_circ), [0.2 0.5 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'b', 'LineWidth', 1.5);
    quiver(0, 0, 0.5*cos(theta_rot), 0.5*sin(theta_rot), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.9);
    
    labels = {
        2.00*S, -0.45*S, '1. Dotted Line';
        4.30*S,  0.35*S, '2. 90^\circ Intersection';
        6.80*S, -0.65*S, '3. Long Zig-Zag';
       10.50*S,  0.35*S, '4. Sweeping Curve';
       11.60*S,  3.30*S, '5. S-Bend Chicane';
        9.85*S,  6.95*S, '6. 180^\circ Hairpin';
        9.10*S,  4.10*S, '7a. Sharp 90^\circ';
        6.50*S,  4.45*S, '7b. Sharp 45^\circ Turns';
        1.50*S, -3.35*S, '8. Return Loop'
    };
    for k = 1:size(labels, 1)
        pt_rot = R_mat * [labels{k,1}; labels{k,2}];
        text(pt_rot(1), pt_rot(2), labels{k,3}, 'FontSize', 9, 'FontWeight', 'bold', ...
             'BackgroundColor', [1 1 0.85], 'EdgeColor', [0.6 0.6 0.6], 'Margin', 2);
    end
    title('Enlarged 8-Feature DDMR Course (Blue Disk = Actual Robot Chassis Scale)');
    xlabel('X (m)'); ylabel('Y (m)');
end

function export_track_to_stl(segments, half_w, thickness, filename)
    V_all = [];
    F_all = [];
    v_offset = 0;
    
    for s = 1:length(segments)
        pts = segments{s};
        N = size(pts, 1);
        if N < 2, continue; end
        
        dx = gradient(pts(:,1));
        dy = gradient(pts(:,2));
        len = hypot(dx, dy);
        len(len == 0) = 1;
        nx = -dy ./ len;
        ny =  dx ./ len;
        
        xL = pts(:,1) + half_w * nx;  yL = pts(:,2) + half_w * ny;
        xR = pts(:,1) - half_w * nx;  yR = pts(:,2) - half_w * ny;
        
        z0 = zeros(N, 1);
        z1 = thickness * ones(N, 1);
        
        V_seg = [xL, yL, z0;
                 xR, yR, z0;
                 xR, yR, z1;
                 xL, yL, z1];
             
        F_seg = [];
        for i = 1:(N-1)
            bL1 = i;       bL2 = i+1;
            bR1 = N+i;     bR2 = N+i+1;
            tR1 = 2*N+i;   tR2 = 2*N+i+1;
            tL1 = 3*N+i;   tL2 = 3*N+i+1;
            
            F_seg = [F_seg;
                     tL1, tR1, tR2;  tL1, tR2, tL2;
                     bL1, bR2, bR1;  bL1, bL2, bR2;
                     bL1, tL1, tL2;  bL1, tL2, bL2;
                     bR1, bR2, tR2;  bR1, tR2, tR1];
        end
        F_seg = [F_seg;
                 1, N+1, 2*N+1;      1, 2*N+1, 3*N+1;
                 N, 3*N, 2*N;        N, 4*N,   3*N];
             
        V_all = [V_all; V_seg]; %#ok<AGROW>
        F_all = [F_all; F_seg + v_offset]; %#ok<AGROW>
        v_offset = size(V_all, 1);
    end
    
    TR = triangulation(F_all, V_all);
    stlwrite(TR, filename);
    disp(['Successfully generated enlarged 3D CAD track file: ', fullfile(pwd, filename)]);
end