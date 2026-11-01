# 🤖 DDMR Control: PID vs State-Space — MATLAB/Simulink

> **Mohammed Emad Said Soliman** | UHCI4H  
> Supervisor: **Kis Károly Árpád** | Dr. **Hajdu Sándor**  
> Mechatronics Engineering BSc. — University of Debrecen  
> Faculty of Engineering | Department of Electrical Engineering and Mechatronics

---

## 📋 Overview

This project presents a **comparative study of PID and State-Space pole-placement control strategies** for a **Differential Drive Mobile Robot (DDMR)**, implemented through **progressive modeling and simulation** in MATLAB/Simulink.

The robot follows a custom figure-8 track using a 6-sensor IR array. The study models the system from a simple open-loop kinematic model all the way up to a full Simscape physical model with a DC motor and H-Bridge motor driver — comparing how PID and State-Space controllers perform at each level of complexity.

---

## 👥 Authors

| Role | Name |
|------|------|
| **Student** | Mohammed Emad Said Soliman (UHCI4H) |
| **Supervisor** | Kis Károly Árpád |
| **Co-supervisor** | Dr. Hajdu Sándor |
| **Institution** | University of Debrecen, Faculty of Engineering |

---

## 🎯 Control Strategies Compared

### 1. PID Control (`DDMR_PID.slx`)
- Separate PID loops for lateral error and heading error
- Tuned for smooth cornering on the figure-8 track
- Applied at both simplified and full motor/H-Bridge model levels

### 2. State-Space Pole Placement (`DDMR_SS.slx`)
- Full 3-state model: `[|e_y|_brake, e_y, e_theta]`
- Inputs: `[v_brake, omega]`
- Controller gain matrix **K** via pole placement (`[-4, -5]` steering poles, `-1.5` braking pole)
- Luenberger observer with poles `[-10, -15]`

### 3. LQR Optimal Control (`LQR.m`)
- Upgraded from pole placement to **Linear Quadratic Regulator**
- Tuned weights: `q_brake=2.25`, `q_ey=4500`, `q_etheta=65`
- Kalman-based observer gain `L_obs` via `lqr(A', C', Q_obs, R_obs)`
- Achieves fast tracking while filtering H-Bridge lag

---

## 📈 Progressive Modeling Stages

| Stage | Model File | Description |
|-------|-----------|-------------|
| 1 | `DDMR_OoenLoop.slx` | Open-loop kinematic model — no control |
| 2 | `DDMR_PID.slx` | PID control — pure kinematic model |
| 3 | `DDMR_PID_Simplified.slx` | Simplified PID |
| 4 | `DDMR_SS.slx` | State-Space pole placement — kinematic |
| 5 | `DDMR_Simscape.slx` | Simscape physical model — no control |
| 6 | `DDMR_Simscape_PID.slx` | Simscape + PID |
| 7 | `DDMR_Simscape_SS.slx` | Simscape + State-Space |
| 8 | `DDMR_Simscape_PID_Motor.slx` | Simscape + PID + DC motor model |
| 9 | `DDMR_Simscape_SS_Motor.slx` | Simscape + State-Space + DC motor model |
| 10 | `DDMR_Simscape_PID_Motor_HBridge.slx` | **Full model** — PID + Motor + H-Bridge |
| 11 | `DDMR_Simscape_SS_Motor_HBridge.slx` | **Full model** — State-Space + Motor + H-Bridge |
| 12 | `DDMR_Simscape_PID_OpenLoop.slx` | Simscape PID open-loop reference |

---

## ⚙️ Robot Parameters

| Parameter | Value |
|-----------|-------|
| Wheel radius (R) | 0.05 m |
| Wheelbase (L) | 0.20 m |
| Base cruising speed (V_ref) | 0.3 m/s |
| Sensor count | 6 IR sensors |
| Sensor forward distance (L_f) | 0.20 m |
| Sensor array width (W_spread) | 0.15 m |
| Line detection radius | 0.03 m |

### DC Motor (Faulhaber)
| Parameter | Value |
|-----------|-------|
| Nominal voltage | 6 V |
| Armature resistance | 2 Ω |
| No-load speed | 105 rad/s |
| No-load current | 0.04 A |
| Stall torque | 0.08 N·m |

---

## 📐 System Model

**States:** `x = [|e_y|_brake ; e_y ; e_theta]`  
**Inputs:** `u = [v_brake ; omega]`

```
A = [0,  0,      0    ]      B = [1, 0]
    [0,  0,  -V_ref   ]          [0, 0]
    [0,  0,      0    ]          [0, 1]
```

**Steering subsystem (for observer):**
```
A_steer = [0, -V_ref]      B_steer = [0]      C_steer = [1, 0]
           [0,    0  ]                 [1]
```

---

## 🛤️ Track

- Custom **figure-8 / lemniscate** track (`black_track.stl`, `challenging_track.stl`)
- Defined parametrically in `Track.m` and `Track2.m`
- Track texture: `track_texture.png`

---

## 📁 File Structure

```
├── 📊 MATLAB Scripts
│   ├── Track.m                              — State-Space gains + track setup
│   ├── Track2.m                             — Extended track with LQR
│   ├── LQR.m                               — LQR & observer gain computation
│   └── simu.m                              — Simulink model builder script
│
├── 🔧 Simulink Models (.slx)
│   ├── DDMR_OoenLoop.slx                   — Stage 1: Open-loop kinematics
│   ├── DDMR_PID.slx                        — Stage 2: PID control
│   ├── DDMR_PID_Simplified.slx             — Stage 3: Simplified PID
│   ├── DDMR_SS.slx                         — Stage 4: State-Space control
│   ├── DDMR_Simscape.slx                   — Stage 5: Simscape physical model
│   ├── DDMR_Simscape_PID.slx               — Stage 6: Simscape + PID
│   ├── DDMR_Simscape_SS.slx                — Stage 7: Simscape + State-Space
│   ├── DDMR_Simscape_PID_Motor.slx         — Stage 8: + DC motor
│   ├── DDMR_Simscape_SS_Motor.slx          — Stage 9: + DC motor (SS)
│   ├── DDMR_Simscape_PID_Motor_HBridge.slx — Stage 10: Full PID model ✅
│   ├── DDMR_Simscape_SS_Motor_HBridge.slx  — Stage 11: Full SS model ✅
│   └── DDMR_Simscape_PID_OpenLoop.slx      — Reference open-loop
│
├── 🗺️ Track Files
│   ├── black_track.stl                     — Simple track (3D)
│   ├── challenging_track.stl               — Complex figure-8 track (3D)
│   └── track_texture.png                   — Track texture image
│
├── 📄 Documents
│   ├── summery.docx / summery.pdf          — Research paper / thesis summary
│   └── Project of mechatronics.pptx / .pdf — Project presentation
│
└── 🎬 videos/                              — Simulation recordings (not tracked by Git)
    ├── DDMR_Simscape_PID.avi
    ├── DDMR_Simscape_PID_Motor.avi
    ├── DDMR_Simscape_PID_Motor_HBridge.avi
    ├── DDMR_Simscape_SS.avi
    ├── DDMR_Simscape_SS_Motor.avi
    └── DDMR_Simscape_SS_Motor_HBridge.avi
```

> **⚠️ Note on Videos:** Simulation recordings (`.avi`) are excluded from Git due to their large size (~540 MB total). They can be generated by running the corresponding `.slx` models in MATLAB/Simulink.

---

## 🚀 How to Run

### Requirements
- MATLAB R2024a or later
- Simulink
- Simscape & Simscape Multibody toolboxes

### Steps
1. Clone the repository
2. Open MATLAB and navigate to the project folder
3. Run `Track.m` (pole placement) or `LQR.m` (optimal control) to initialize workspace variables
4. Open the desired `.slx` model and click **Run**

**Recommended starting points:**
- `DDMR_Simscape_SS_Motor_HBridge.slx` — Full State-Space simulation
- `DDMR_Simscape_PID_Motor_HBridge.slx` — Full PID simulation

---

## 📊 Key Results

| Controller | Model Level | Track Following |
|-----------|-------------|----------------|
| PID | Kinematic | ✅ Good |
| State-Space (PP) | Kinematic | ✅ Good |
| PID | + Simscape + Motor | ✅ Stable |
| State-Space (PP) | + Simscape + Motor | ✅ Stable |
| LQR | + H-Bridge + Motor | ✅ Best performance |

---

*Mechatronics Engineering BSc. — University of Debrecen, 2026*
