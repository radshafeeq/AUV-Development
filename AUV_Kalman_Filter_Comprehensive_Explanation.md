# Comprehensive Analysis: Kalman Filter in the AUV Visual Servoing Control System

> **Author**: Radhi Shafeeq  
> **Context**: Undergraduate Mechatronics Engineering Thesis — Hasanuddin University  
> **System**: BlueROV2 Heavy (6-DOF) and BlueROV2 Standard (4-DOF) with YOLO-based Visual Servoing & Kalman State Estimation

---

## Table of Contents

1. [Introduction & Motivation](#1-introduction--motivation)
2. [AUV Kinematics — How the Vehicle Moves](#2-auv-kinematics--how-the-vehicle-moves)
3. [AUV Dynamics — What Forces Drive the Motion](#3-auv-dynamics--what-forces-drive-the-motion)
4. [The Core Problem: Why Raw Vision Fails](#4-the-core-problem-why-raw-vision-fails)
5. [Kalman Filter Theory — The Mathematical Foundation](#5-kalman-filter-theory--the-mathematical-foundation)
6. [The State-Space Model in Our AUV](#6-the-state-space-model-in-our-auv)
7. [The Two-Step Recursive Cycle: Predict → Correct](#7-the-two-step-recursive-cycle-predict--correct)
8. [Noise Covariance Matrices: Q, R, and P](#8-noise-covariance-matrices-q-r-and-p)
9. [How the Kalman Filter Connects to AUV Kinematics & Dynamics](#9-how-the-kalman-filter-connects-to-auv-kinematics--dynamics)
10. [Occlusion Handling & Dead-Reckoning Prediction](#10-occlusion-handling--dead-reckoning-prediction)
11. [Complete Closed-Loop Signal Flow](#11-complete-closed-loop-signal-flow)
12. [Numerical Example: 5-Cycle Walk-Through](#12-numerical-example-5-cycle-walk-through)
13. [Implementation Mapping to Code](#13-implementation-mapping-to-code)
14. [Parameter Taxonomy Dictionary](#14-parameter-taxonomy-dictionary)
15. [Summary & Key Takeaways](#15-summary--key-takeaways)
16. [References](#16-references)

---

## 1. Introduction & Motivation

An Autonomous Underwater Vehicle (AUV) that performs **visual target tracking** faces a fundamentally different challenge than land or aerial robots. Underwater, the camera observes a target through a turbid, refractive, poorly-lit medium. Every raw pixel measurement from the YOLO object detector is corrupted by:

- **Detection jitter** — bounding box centres fluctuate by 5–20 px frame-to-frame even when the target is stationary
- **Intermittent occlusion** — bubbles, suspended particles, light refraction, or temporary loss-of-view cause the detector to output **zero detections** for several consecutive frames
- **Latency** — the camera-to-GPU inference pipeline introduces a 30–100 ms delay; the AUV has already moved by the time the detection result arrives

If the thruster controller acts directly on these raw, noisy, intermittent measurements, the AUV will exhibit:

1. **Jerky, oscillating motion** — thrusters rapidly switching direction trying to follow pixel noise
2. **Complete loss of tracking** during occlusion — the AUV stops and drifts with no target information
3. **Phase lag** — control actions based on stale measurements that do not represent the current state

The **Kalman Filter** solves all three problems simultaneously by providing an **optimal, mathematically principled estimate** of where the target *actually is* (and where it is *going*), even when measurements are noisy or missing.

---

## 2. AUV Kinematics — How the Vehicle Moves

### 2.1 Coordinate Frames

Following the SNAME (Society of Naval Architects and Marine Engineers) convention adopted by Fossen (2011), we define two coordinate frames:

| Frame | Symbol | Description |
|-------|--------|-------------|
| **Earth-Fixed (Inertial)** | $\{n\}$ | NED frame (North-East-Down), fixed to the Earth's surface. Gravity acts along the positive $z_n$-axis. |
| **Body-Fixed** | $\{b\}$ | Origin at the AUV's centre of gravity (CG), moves and rotates with the vehicle. $x_b$ points forward (bow), $y_b$ points starboard, $z_b$ points downward. |

### 2.2 The Six Degrees of Freedom

A rigid body submerged in water has six degrees of freedom (DOF). Using the SNAME notation:

| DOF | Motion Type | Body-Frame Velocity | Earth-Frame Position/Angle | Force/Moment |
|-----|-------------|---------------------|----------------------------|--------------|
| 1 — Surge | Translation along $x_b$ | $u$ | $x$ | $X$ |
| 2 — Sway | Translation along $y_b$ | $v$ | $y$ | $Y$ |
| 3 — Heave | Translation along $z_b$ | $w$ | $z$ | $Z$ |
| 4 — Roll | Rotation about $x_b$ | $p$ | $\phi$ | $K$ |
| 5 — Pitch | Rotation about $y_b$ | $q$ | $\theta$ | $M$ |
| 6 — Yaw | Rotation about $z_b$ | $r$ | $\psi$ | $N$ |

> [!IMPORTANT]
> **BlueROV2 Heavy** has **6 controllable DOF** (surge, sway, heave, roll, pitch, yaw) with 8 thrusters (`vectored_6dof`). 
> **BlueROV2 Standard** operates as a decoupled **4-DOF** system (surge, sway, heave, yaw) with 6 thrusters (`vectored`), where roll ($\phi$) and pitch ($\theta$) are passively stabilized by positive metacentric height ($GM_T > 0$).

### 2.3 Kinematic Vectors

We define the generalised position and velocity vectors:

$$\boldsymbol{\eta} = \begin{bmatrix} \boldsymbol{\eta}_1 \\ \boldsymbol{\eta}_2 \end{bmatrix} = \begin{bmatrix} x & y & z & \phi & \theta & \psi \end{bmatrix}^T$$

$$\boldsymbol{\nu} = \begin{bmatrix} \boldsymbol{\nu}_1 \\ \boldsymbol{\nu}_2 \end{bmatrix} = \begin{bmatrix} u & v & w & p & q & r \end{bmatrix}^T$$

### 2.4 The Kinematic Equation

The relationship between earth-frame position rate-of-change and body-frame velocities is:

$$\dot{\boldsymbol{\eta}} = \mathbf{J}(\boldsymbol{\eta}_2) \, \boldsymbol{\nu}$$

where $\mathbf{J}(\boldsymbol{\eta}_2)$ is the **6×6 Jacobian transformation matrix** composed of two sub-matrices:

$$\mathbf{J}(\boldsymbol{\eta}_2) = \begin{bmatrix} \mathbf{R}(\phi, \theta, \psi) & \mathbf{0}_{3\times3} \\ \mathbf{0}_{3\times3} & \mathbf{T}(\phi, \theta) \end{bmatrix}$$

**$\mathbf{R}(\phi, \theta, \psi)$** is the rotation matrix (using ZYX Euler angles) that transforms linear velocities from the body frame to the earth frame:

$$\mathbf{R} = \begin{bmatrix} c\psi c\theta & c\psi s\theta s\phi - s\psi c\phi & c\psi s\theta c\phi + s\psi s\phi \\ s\psi c\theta & s\psi s\theta s\phi + c\psi c\phi & s\psi s\theta c\phi - c\psi s\phi \\ -s\theta & c\theta s\phi & c\theta c\phi \end{bmatrix}$$

where $c(\cdot) = \cos(\cdot)$ and $s(\cdot) = \sin(\cdot)$.

**$\mathbf{T}(\phi, \theta)$** transforms angular velocities from body to earth frame:

$$\mathbf{T} = \begin{bmatrix} 1 & s\phi \tan\theta & c\phi \tan\theta \\ 0 & c\phi & -s\phi \\ 0 & s\phi / c\theta & c\phi / c\theta \end{bmatrix}$$

> [!NOTE]
> **Physical meaning**: Kinematics tells us *how position changes* given a velocity, but says nothing about *what forces produce that velocity*. That is the domain of dynamics.

---

## 3. AUV Dynamics — What Forces Drive the Motion

### 3.1 Fossen's 6-DOF Equation of Motion

The dynamics of a rigid body moving through a viscous fluid are described by Fossen's vector equation:

$$\mathbf{M} \dot{\boldsymbol{\nu}} + \mathbf{C}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{D}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{g}(\boldsymbol{\eta}) = \boldsymbol{\tau}$$

Each term represents a distinct physical phenomenon:

### 3.2 Term-by-Term Breakdown

#### $\mathbf{M}\dot{\boldsymbol{\nu}}$ — Inertial Forces

$$\mathbf{M} = \mathbf{M}_{RB} + \mathbf{M}_A$$

| Component | Description |
|-----------|-------------|
| $\mathbf{M}_{RB}$ | **Rigid-body inertia matrix** (6×6). Depends on the AUV's mass $m = 13.5\,\text{kg}$ and rigid moments of inertia $I_{xx}=0.16, I_{yy}=0.21, I_{zz}=0.245$. Diagonal elements represent resistance to linear and angular acceleration. |
| $\mathbf{M}_A$ | **Added mass matrix** (6×6). Represents virtual mass of entrained fluid. For BlueROV2: $X_{\dot{u}}=-6.36$, $Y_{\dot{v}}=-7.12$, $Z_{\dot{w}}=-18.68$, $K_{\dot{p}}=-0.015$, $M_{\dot{q}}=-0.080$, $N_{\dot{r}}=-0.245$. |

For the BlueROV2 ($m = 13.5\,\text{kg}$, $I_{xx} = 0.16$, $I_{yy} = 0.21$, $I_{zz} = 0.245$):

$$\mathbf{M}_{RB} = \begin{bmatrix} 13.5 & 0 & 0 & 0 & 0 & 0 \\ 0 & 13.5 & 0 & 0 & 0 & 0 \\ 0 & 0 & 13.5 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0.16 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0.21 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0.245 \end{bmatrix}$$

#### $\mathbf{C}(\boldsymbol{\nu})\boldsymbol{\nu}$ — Coriolis and Centripetal Forces

When the AUV rotates while translating, Coriolis and centripetal forces appear. These velocity-dependent coupling terms produce complex non-linear behaviors. Most notably, the **Munk Moment** $(X_{\dot{u}} - Y_{\dot{v}})u_r v_r$ generates a destabilizing yaw torque (+0.76 kg gain for BlueROV2) when the vehicle experiences combined surge and sway, attempting to turn the vehicle broadside to the flow. $\mathbf{C}$ is a function of $\boldsymbol{\nu}$ and is skew-symmetric.

#### $\mathbf{D}(\boldsymbol{\nu})\boldsymbol{\nu}$ — Hydrodynamic Damping (Drag)

This is the dominant resistance force underwater. It has two components:

$$\mathbf{D}(\boldsymbol{\nu}) = \mathbf{D}_l + \mathbf{D}_q(\boldsymbol{\nu})$$

| Component | Form | Description |
|-----------|------|-------------|
| **Linear drag** $\mathbf{D}_l$ | $\mathbf{D}_l \boldsymbol{\nu}$ | Proportional to velocity. Dominates at low speeds (skin friction). |
| **Quadratic drag** $\mathbf{D}_q$ | $\mathbf{D}_q(\boldsymbol{\nu})\boldsymbol{\nu}$ | Proportional to $|\nu|\nu$. Dominates at moderate-to-high speeds (pressure drag). |

From the hydrodynamic study, the linear and quadratic drag coefficients are:

| Coefficient | Value | Physical Meaning |
|-------------|-------|------------------|
| $X_{u|u|}$ | $-18.18$ | Surge quadratic drag — resistance to forward/backward motion |
| $Y_{v|v|}$ | $-21.66$ | Sway quadratic drag — resistance to lateral motion |
| $Z_{w|w|}$ | $-36.99$ | Heave quadratic drag — resistance to vertical motion (largest projected area) |
| $N_{r|r|}$ | $-1.55$ | Yaw quadratic drag — resistance to rotation |

(Linear skin friction values are $X_u = -4.03$, $Y_v = -6.22$, $Z_w = -5.18$, $N_r = -0.50$).

> [!TIP]
> **Why sway drag > surge drag**: The AUV hull is elongated along the surge axis. Moving sideways presents a much larger frontal area to the water, creating greater resistance. This asymmetry is critical for understanding why the AUV turns (yaws) rather than translates sideways when the controller applies a correction.

#### $\mathbf{g}(\boldsymbol{\eta})$ — Gravitational and Buoyancy Restoring Forces

$$\mathbf{g}(\boldsymbol{\eta}) = \begin{bmatrix} (W - B)\sin\theta \\ -(W - B)\cos\theta\sin\phi \\ -(W - B)\cos\theta\cos\phi \\ -\overline{BG}_z B \cos\theta\sin\phi \\ -\overline{BG}_z B \sin\theta \\ 0 \end{bmatrix}$$

where:
- $W = mg = 13.5 \times 9.81 = 132.44\,\text{N}$ is the weight force
- $B = \rho_{water} \cdot \nabla \cdot g = 134.74\,\text{N}$ is the hydrostatic buoyant force
- $GM_T = z_g - z_b$ is the vertical metacentric height between CB and CG

> [!IMPORTANT]
> In the BlueROV2, the heavy components (ballast/battery) are mounted low while syntactic foam is mounted high, establishing a positive metacentric height ($GM_T = 0.020\,\text{m}$). This acts as a physical torsional spring ($k_\phi = z_g W$). In the **BlueROV2 Standard (4-DOF)**, this buoyancy-gravity couple passively rightens the vehicle ($\phi \to 0, \theta \to 0$), leaving roll and pitch unactuated.

#### $\boldsymbol{\tau}$ — Thruster Forces and Moments

The control input vector produced by the 6 thrusters:

$$\boldsymbol{\tau} = \begin{bmatrix} X_{thrust} \\ Y_{thrust} \\ Z_{thrust} \\ K_{thrust} \\ M_{thrust} \\ N_{thrust} \end{bmatrix} = \mathbf{T}_{config} \cdot \mathbf{f}$$

where $\mathbf{T}_{config}$ is the **thruster configuration matrix** mapping individual thruster forces $\mathbf{f}$ to body-frame forces/moments based on each thruster's physical geometry.

For the BlueROV2 platforms:
- **BlueROV2 Heavy (8 thrusters)**: $\mathbf{T}_{6 \times 8}$ pseudo-inverse matrix handles full 6-DOF actuation (4 horizontal vectored + 4 vertical corner thrusters).
- **BlueROV2 Standard (6 thrusters)**: $\mathbf{T}_{4 \times 6}$ pseudo-inverse matrix handles decoupled 4-DOF actuation (Surge, Sway, Heave, Yaw).

---

## 4. The Core Problem: Why Raw Vision Fails

### 4.1 The Visual Servoing Architecture

Our AUV uses **Image-Based Visual Servoing (IBVS)**. The camera is body-fixed, looking forward. The control law operates directly in the **image plane** (pixel coordinates), not in the 3D world frame:

```
Camera Frame → YOLO Detection → Pixel Error → Controller → Thruster Commands
```

The target's position in the image is:

$$\mathbf{s} = \begin{bmatrix} u_{target} \\ v_{target} \end{bmatrix} \quad \text{(pixel coordinates)}$$

The desired position is the frame centre:

$$\mathbf{s}^* = \begin{bmatrix} u_{center} \\ v_{center} \end{bmatrix} = \begin{bmatrix} w/2 \\ h/2 \end{bmatrix}$$

The image error vector is:

$$\mathbf{e} = \mathbf{s} - \mathbf{s}^* = \begin{bmatrix} u_{target} - w/2 \\ v_{target} - h/2 \end{bmatrix}$$

The proportional controller then maps this error to thruster commands:

$$\tau_{yaw} = K_{p,yaw} \cdot e_x, \qquad \tau_{heave} = K_{p,heave} \cdot e_y$$

### 4.2 The Three Failure Modes Without a Kalman Filter

```mermaid
graph TD
    A["Raw YOLO Detection<br/>z = [u, v] pixels"] --> B{"Is detection<br/>available?"}
    B -->|Yes| C["Noisy measurement<br/>±5-20px jitter"]
    B -->|No| D["ZERO information<br/>AUV stops dead"]
    C --> E["Controller sees<br/>jittering error signal"]
    E --> F["Thrusters oscillate<br/>violently at 30 Hz"]
    D --> G["Controller has no<br/>error to compute"]
    G --> H["AUV drifts with<br/>ocean current"]
    
    style F fill:#ff4444,color:#fff
    style H fill:#ff4444,color:#fff
```

**Failure 1 — Measurement Jitter → Thruster Oscillation**

Even when tracking a stationary target, consecutive YOLO detections fluctuate:

| Frame | Raw $u$ (px) | Raw $v$ (px) | $\Delta u$ | $\Delta v$ |
|-------|------------|------------|-----------|-----------|
| $k$ | 318 | 242 | — | — |
| $k+1$ | 325 | 237 | +7 | −5 |
| $k+2$ | 314 | 245 | −11 | +8 |
| $k+3$ | 329 | 233 | +15 | −12 |

The controller interprets each fluctuation as real target motion and commands the thrusters to compensate. At 30 fps, this creates a 30 Hz oscillation in thruster commands — mechanical vibration, wasted energy, and acoustic noise.

**Failure 2 — Occlusion → Complete Track Loss**

When bubbles, murky water, or light glare cause YOLO to return zero detections, the controller receives **no error signal**. It cannot command any correction. The AUV drifts passively with any ambient current, and when the target reappears, it may have moved far from the frame centre, causing a sudden large correction that overshoots.

**Failure 3 — Latency → Phase Lag**

The camera-to-controller pipeline has ~33 ms latency (1 frame at 30 fps). If the target moves at 50 px/s, by the time the controller acts on frame $k$, the target has already moved 1.7 px. The controller is always "chasing" the target's past position.

---

## 5. Kalman Filter Theory — The Mathematical Foundation

### 5.1 What Is the Kalman Filter?

The Kalman Filter (Kalman, 1960) is a **recursive, optimal state estimator** for linear dynamical systems with Gaussian noise. "Optimal" means it minimises the mean squared error of the state estimate, given:

1. A **process model** describing how the system evolves over time
2. A **measurement model** describing how observations relate to the state
3. Statistical characterisations of the **process noise** and **measurement noise**

### 5.2 Why "Optimal"?

Among all possible linear estimators, the Kalman Filter produces the estimate with the **minimum variance** (smallest uncertainty). For Gaussian noise, this is also the **maximum likelihood** and **maximum a posteriori** estimate. No linear filter can do better.

### 5.3 Why Not Just Average?

A simple moving average (e.g., average the last 5 detections) also smooths noise, but it:

- Introduces **fixed latency** (the average lags behind the true position)
- Cannot **predict** during occlusion (it has no motion model)
- Treats all measurements equally (cannot weight high-confidence detections more)
- Cannot estimate **velocity** (only position)

The Kalman Filter addresses all of these by incorporating a **physics-based motion model** and **adaptive weighting** based on uncertainty.

---

## 6. The State-Space Model in Our AUV

### 6.1 State Vector

We model the target's motion in the image plane with a **4-dimensional state vector**:

$$\mathbf{x}_k = \begin{bmatrix} x_k \\ y_k \\ v_{x,k} \\ v_{y,k} \end{bmatrix} = \begin{bmatrix} \text{target horizontal position (px)} \\ \text{target vertical position (px)} \\ \text{target horizontal velocity (px/s or px/frame)} \\ \text{target vertical velocity (px/s or px/frame)} \end{bmatrix}$$

### 6.2 Process Model (State Transition)

We assume a **constant-velocity motion model**: between frames, the target moves at approximately constant velocity. This is the discrete-time kinematic equation:

$$\mathbf{x}_{k} = \mathbf{A} \mathbf{x}_{k-1} + \mathbf{w}_{k-1}$$

where the **state transition matrix** $\mathbf{A}$ encodes the constant-velocity kinematics:

$$\mathbf{A} = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

with $\Delta t = 0.033\,\text{s}$ (one frame at 30 fps).

**Expanding the matrix multiplication explicitly**:

$$\begin{bmatrix} x_k \\ y_k \\ v_{x,k} \\ v_{y,k} \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0.03333 & 0 \\ 0 & 1 & 0 & 0.03333 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_{k-1} \\ y_{k-1} \\ v_{x,k-1} \\ v_{y,k-1} \end{bmatrix}$$

This gives the intuitive kinematic equations:

$$x_k = x_{k-1} + v_{x,k-1} \cdot \Delta t + w_{x,k-1}$$
$$y_k = y_{k-1} + v_{y,k-1} \cdot \Delta t + w_{y,k-1}$$
$$v_{x,k} = v_{x,k-1} + w_{vx,k-1}$$
$$v_{y,k} = v_{y,k-1} + w_{vy,k-1}$$

> [!NOTE]
> **Connection to AUV kinematics**: This is the discrete, 2D image-plane analogue of the kinematic equation $\dot{\boldsymbol{\eta}} = \mathbf{J}\boldsymbol{\nu}$. In the full 6-DOF formulation, position updates depend on velocity through the Jacobian. Here, in the image plane, the relationship simplifies to linear translation because we track pixel coordinates directly.

### 6.3 Measurement Model

The camera provides only **position** measurements (the bounding box centre from YOLO). Velocity is not directly measured — it must be **inferred** by the filter. The measurement equation is:

$$\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k$$

where the **measurement matrix** $\mathbf{H}$ extracts only the position components:

$$\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$

and $\mathbf{z}_k = [z_x, z_y]^T$ is the raw YOLO bounding box centre in pixels.

**What $\mathbf{H}$ means physically**: The camera can see *where* the target is, but cannot directly see *how fast* it is moving. The Kalman Filter cleverly infers velocity by observing how position changes over time.

### 6.4 Noise Models

$$\mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{Q}) \qquad \text{(process noise — model uncertainty)}$$
$$\mathbf{v}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{R}) \qquad \text{(measurement noise — sensor uncertainty)}$$

Both are assumed to be **zero-mean Gaussian** and **mutually uncorrelated**.

---

## 7. The Two-Step Recursive Cycle: Predict → Correct

Every frame, the Kalman Filter executes exactly two steps. This is the core algorithm:

### 7.1 Step 1 — PREDICT (Time Update / Propagation)

*"Where do I think the target will be, based on physics alone?"*

**Predicted State Estimate** (a priori):

$$\hat{\mathbf{x}}_{k|k-1} = \mathbf{A} \hat{\mathbf{x}}_{k-1|k-1}$$

**Predicted Error Covariance** (a priori):

$$\mathbf{P}_{k|k-1} = \mathbf{A} \mathbf{P}_{k-1|k-1} \mathbf{A}^T + \mathbf{Q}$$

> **Physical interpretation**: We propagate the last known state forward in time using the kinematic model. The uncertainty ($\mathbf{P}$) **grows** because we are extrapolating — the longer we go without a measurement, the less certain we become. $\mathbf{Q}$ quantifies how much the constant-velocity assumption can be wrong.

### 7.2 Step 2 — CORRECT (Measurement Update)

*"Now I have a new YOLO detection. How do I combine it with my prediction?"*

**Innovation (Measurement Residual)**:

$$\tilde{\mathbf{y}}_k = \mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_{k|k-1}$$

This is the difference between what we *measured* and what we *predicted*. A large innovation means the measurement is surprising.

**Innovation Covariance**:

$$\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}$$

**Kalman Gain**:

$$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T \mathbf{S}_k^{-1}$$

> [!IMPORTANT]
> **The Kalman Gain $\mathbf{K}$ is the key insight of the entire filter.** It is a matrix that determines how much to trust the new measurement versus the prediction:
> - If $\mathbf{R}$ is large (noisy sensor) → $\mathbf{K}$ is small → trust the prediction more
> - If $\mathbf{P}$ is large (uncertain model) → $\mathbf{K}$ is large → trust the measurement more
> - The gain is **computed automatically** at each frame based on the current uncertainty — no manual tuning of this balance is needed.

**Corrected State Estimate** (a posteriori):

$$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \tilde{\mathbf{y}}_k$$

**Corrected Error Covariance** (a posteriori):

$$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}$$

> **Physical interpretation**: The corrected state is a **weighted blend** of the prediction and the measurement. The uncertainty ($\mathbf{P}$) **shrinks** after incorporating a measurement — we are now more certain about the target's state.

### 7.3 The Predict–Correct Cycle Visualised

```mermaid
graph LR
    subgraph "Frame k-1"
        A["Corrected State<br/>x̂(k-1|k-1)<br/>P(k-1|k-1)"]
    end
    
    subgraph "Frame k: PREDICT"
        B["Predicted State<br/>x̂(k|k-1) = A · x̂(k-1|k-1)<br/>P(k|k-1) = A·P·Aᵀ + Q"]
    end
    
    subgraph "Frame k: CORRECT"
        C["YOLO Detection<br/>z(k) = [u, v]"]
        D["Kalman Gain<br/>K = P·Hᵀ·S⁻¹"]
        E["Corrected State<br/>x̂(k|k) = x̂(k|k-1) + K·(z - H·x̂)<br/>P(k|k) = (I - K·H)·P"]
    end
    
    A -->|"Kinematic<br/>propagation"| B
    B --> D
    C --> D
    D --> E
    E -->|"Next frame"| A
    
    style B fill:#2196F3,color:#fff
    style E fill:#4CAF50,color:#fff
    style C fill:#FF9800,color:#fff
```

---

## 8. Noise Covariance Matrices: Q, R, and P

### 8.1 Process Noise $\mathbf{Q}$ — Continuous White Noise Acceleration (CWNA)

To accurately model unmodeled target accelerations, we treat the process noise as a Continuous White Noise Acceleration (CWNA) model with power spectral density $q_s = 0.05$. The continuous-time system matrix $\mathbf{F}_c$ and noise input matrix $\mathbf{G}_c$ are:

$$\mathbf{F}_c = \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \end{bmatrix}, \quad \mathbf{G}_c = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ 1 & 0 \\ 0 & 1 \end{bmatrix}$$

The discrete process noise covariance matrix $\mathbf{Q} \in \mathbb{R}^{4\times4}$ is derived via the integral:

$$\mathbf{Q} = \int_{0}^{\Delta t} e^{\mathbf{F}_c \tau} \mathbf{G}_c q_s \mathbf{G}_c^T e^{\mathbf{F}_c^T \tau} d\tau = q_s \begin{bmatrix} \frac{\Delta t^3}{3} & 0 & \frac{\Delta t^2}{2} & 0 \\ 0 & \frac{\Delta t^3}{3} & 0 & \frac{\Delta t^2}{2} \\ \frac{\Delta t^2}{2} & 0 & \Delta t & 0 \\ 0 & \frac{\Delta t^2}{2} & 0 & \Delta t \end{bmatrix}$$

For $\Delta t = 0.03333$ s and $q_s = 0.05$, the final non-diagonal $\mathbf{Q}$ matrix is:

$$\mathbf{Q} = \begin{bmatrix} 6.16 \times 10^{-7} & 0 & 2.778 \times 10^{-5} & 0 \\ 0 & 6.16 \times 10^{-7} & 0 & 2.778 \times 10^{-5} \\ 2.778 \times 10^{-5} & 0 & 1.667 \times 10^{-3} & 0 \\ 0 & 2.778 \times 10^{-5} & 0 & 1.667 \times 10^{-3} \end{bmatrix}$$

**Physical meaning**: $\sigma_q = 0.05$ means we expect the constant-velocity model to be accurate to within $\pm 0.05$ px/frame. This is **small**, reflecting our belief that targets underwater generally move smoothly (they do not teleport or make instant 90° turns).

**Effect of tuning $\mathbf{Q}$**:
- $\mathbf{Q}$ too small → filter is overconfident in the model → sluggish response to real motion changes, overshooting on turns
- $\mathbf{Q}$ too large → filter doubts the model → output becomes noisy like the raw measurements, losing the smoothing benefit

### 8.2 Measurement Noise $\mathbf{R}$ — How Noisy the YOLO Detections Are

$$\mathbf{R} = \begin{bmatrix} \sigma_r^2 & 0 \\ 0 & \sigma_r^2 \end{bmatrix} = \begin{bmatrix} 0.20 & 0 \\ 0 & 0.20 \end{bmatrix}$$

**Physical meaning**: The measurement noise covariance matrix captures the YOLO bounding box pixel variance $\sigma_r^2 = 0.20$. This accounts for:
- Bounding box size fluctuations
- Sub-pixel detector inconsistency
- Underwater light refraction distortion

**The ratio $\mathbf{Q}/\mathbf{R}$ controls the filter's behaviour**:

| Ratio $\mathbf{Q}_{ii} / \mathbf{R}_{ii}$ | Filter Behaviour | Analogy |
|------------------------------|-------------------|---------|
| $\ll 1$ (our case: $0.05/0.20 = 0.25$) | **Strongly smoothing** — trusts model, dampens noise | Like a heavy flywheel: stable but slow to respond |
| $\approx 1$ | Balanced — equal trust in model and sensor | Moderate smoothing |
| $\gg 1$ | **Reactive** — trusts sensor, less smoothing | Like raw sensor pass-through |

Our ratio of $0.05/0.2 = 0.25$ produces **strong smoothing**, which is ideal for thruster jitter elimination.

### 8.3 Error Covariance $\mathbf{P}$ — Our Current Uncertainty

$\mathbf{P}$ is initialised as $\mathbf{I}_4$ (identity) and evolves dynamically:

- **After PREDICT**: $\mathbf{P}$ grows (uncertainty increases because we extrapolated)
- **After CORRECT**: $\mathbf{P}$ shrinks (uncertainty decreases because we got new information)
- **During occlusion** (predict-only, no correction): $\mathbf{P}$ grows continuously, reflecting our decreasing confidence

---

## 9. How the Kalman Filter Connects to AUV Kinematics & Dynamics

### 9.1 The Bridge Between Pixel Space and Physical Motion

The Kalman Filter operates in **pixel coordinates**, but its effects propagate directly into the AUV's physical motion through the control loop:

```mermaid
graph TB
    subgraph "IMAGE PLANE (Pixels)"
        A["YOLO Raw Detection<br/>z = [u, v] px"]
        B["Kalman Filter<br/>x̂ = [x, y, vx, vy]"]
        C["Normalised Error<br/>e_x = (x̂ - w/2)/(w/2)<br/>e_y = (ŷ - h/2)/(h/2)"]
    end
    
    subgraph "CONTROL LAW"
        D["P-Controller<br/>τ_yaw = Kp_yaw · e_x · 400<br/>τ_heave = Kp_heave · e_y · 400"]
    end
    
    subgraph "DYNAMICS (Physical)"
        E["MAVLink<br/>manual_control_send()"]
        F["ArduSub PID<br/>Mixer"]
        G["6 Thrusters<br/>T₁...T₆"]
        H["AUV Motion<br/>M·ν̇ + C·ν + D·ν + g = τ"]
    end
    
    subgraph "KINEMATICS (Physical)"
        I["Position Update<br/>η̇ = J(η)·ν"]
        J["Camera Moves<br/>with AUV body"]
    end
    
    A -->|"Noisy<br/>measurement"| B
    B -->|"Smoothed<br/>estimate"| C
    C --> D
    D --> E
    E --> F
    F --> G
    G -->|"Force τ"| H
    H -->|"Velocity ν"| I
    I --> J
    J -->|"New viewpoint<br/>of target"| A
    
    style B fill:#4CAF50,color:#fff
    style H fill:#2196F3,color:#fff
    style I fill:#9C27B0,color:#fff
```

### 9.2 The Kalman Filter's Role at Each Stage

| Stage | Without Kalman Filter | With Kalman Filter |
|-------|----------------------|-------------------|
| **Measurement** | Raw $[u, v]$ with ±15 px jitter | Smoothed $[\hat{x}, \hat{y}]$ with ~±2 px variation |
| **Error Signal** | Oscillates rapidly, sign changes frame-to-frame | Smooth, monotonic convergence toward zero |
| **Yaw Command** | $\tau_{yaw}$ flips between +60 and −60 at 30 Hz | $\tau_{yaw}$ changes gradually: 50 → 40 → 30 → ... |
| **Thruster Response** | Motors whine and vibrate, current spikes, mechanical stress | Smooth thrust ramps, efficient energy use |
| **AUV Trajectory** | Jerky zigzag path with overshoot | Smooth, direct approach to target |
| **During Occlusion** | AUV stops; drifts; loses target | AUV continues on predicted trajectory for up to 0.5 s |

### 9.3 Effect on Each Dynamic Term in the Decoupled 4-DOF State-Space Model

The Kalman Filter operates in pixel space, but its smoothing effect directly improves the stability of the BlueROV2 Standard's decoupled 4-DOF plant. 

> [!NOTE]
> **Connection to the 15-State Navigation EKF**: In a full 3D autonomous underwater navigation context (as detailed in the Kalman Filter monograph), the AUV uses a 15-State EKF fusing IMU, DVL, and USBL data. The complex hydrodynamic forces derived in our kinematic/dynamic study ($\mathbf{M}$, $\mathbf{C}$, $\mathbf{D}$, $\mathbf{g}$) plug *directly* into the non-linear velocity propagation equation of the EKF via the $\boldsymbol{f}_{hydro}^b$ term:
> $$\dot{\boldsymbol{v}}^b = \boldsymbol{a}_{IMU}^b - \boldsymbol{b}_a - \boldsymbol{n}_a - \mathbf{S}(\boldsymbol{\omega}^b - \boldsymbol{b}_g)\boldsymbol{v}^b + \mathbf{R}_n^b(\boldsymbol{q}^n)\boldsymbol{g}^n + \boldsymbol{f}_{hydro}^b$$
> Meanwhile, the visual servoing filter described here is a localized 4D Constant-Velocity filter operating in the image plane to feed smooth error signals to the thruster controller.

**Surge, Sway, and Heave (Translational Dynamics)**: 
Smooth KF error signals prevent rapid thruster oscillations, which is critical because hydrodynamic quadratic drag $\mathbf{D}_q(\boldsymbol{\nu})$ heavily penalizes erratic velocity changes. A sudden spike in sway velocity $v_r$ incurs massive resistance ($Y_{v|v|} = -21.66$), wasting energy. Smooth commands allow efficient cruising. Furthermore, jerky heave commands can disturb the passive pitch equilibrium, causing the vehicle to wobble around its metacentric restoring spring ($k_\theta$). The KF ensures smooth heave transitions, keeping $\theta \approx 0$.

**Yaw (Rotational Dynamics) & The Munk Moment**: 
Perhaps most importantly, erratic yaw/sway commands caused by raw pixel jitter trigger the destabilizing **Munk Moment** $(X_{\dot{u}} - Y_{\dot{v}})u_r v_r$. Because the BlueROV2 has a highly asymmetric added mass profile (transverse added mass $Y_{\dot{v}}$ exceeds surge added mass $X_{\dot{u}}$), combined surge and sway creates a positive, destabilizing yaw torque (+0.76 kg gain). If the vehicle aggressively zig-zags to chase pixel noise, this Munk Moment constantly fights the yaw controller. The KF eliminates pixel jitter, minimizing unnecessary sway $v_r$, which actively suppresses the destabilizing Munk Moment and keeps the AUV tracking straight.

---

## 10. Occlusion Handling & Dead-Reckoning Prediction

### 10.1 What Happens During Occlusion

When YOLO returns zero detections (turbidity, bubbles, glare), the Kalman Filter switches to **predict-only mode** — also known as **dead reckoning** in navigation terminology:

```mermaid
sequenceDiagram
    participant YOLO as YOLO Detector
    participant KF as Kalman Filter
    participant Ctrl as Controller
    participant AUV as AUV Thrusters
    
    Note over YOLO,AUV: Normal Tracking (Predict + Correct)
    YOLO->>KF: Detection z = [320, 240]
    KF->>KF: predict() → x̂ = [319, 241]
    KF->>KF: correct(z) → x̂ = [320, 240]
    KF->>Ctrl: Smoothed [320, 240]
    Ctrl->>AUV: τ_yaw=0, τ_heave=0
    
    Note over YOLO,AUV: Occlusion Begins (Predict Only)
    YOLO->>KF: No detection!
    KF->>KF: predict() → x̂ = [325, 238]
    Note right of KF: Uses last velocity estimate<br/>(vx=+5, vy=-2)
    KF->>KF: handle_missing_frame()<br/>missed_frames = 1
    KF->>Ctrl: Predicted [325, 238]
    Ctrl->>AUV: τ_yaw=+12, τ_heave=-8
    
    YOLO->>KF: Still no detection!
    KF->>KF: predict() → x̂ = [330, 236]
    KF->>KF: handle_missing_frame()<br/>missed_frames = 2
    KF->>Ctrl: Predicted [330, 236]
    Ctrl->>AUV: τ_yaw=+24, τ_heave=-16
    
    Note over YOLO,AUV: ... continues for up to 15 frames (~0.5 s) ...
    
    Note over YOLO,AUV: Target Reappears (Resume Predict + Correct)
    YOLO->>KF: Detection z = [338, 232]
    KF->>KF: predict() → x̂ = [335, 234]
    KF->>KF: correct(z) → x̂ = [337, 233]
    Note right of KF: Smoothly re-acquires!<br/>missed_frames = 0
    KF->>Ctrl: Smoothed [337, 233]
    Ctrl->>AUV: τ_yaw=+30, τ_heave=-20
```

### 10.2 The 15-Frame Safety Limit

After **15 consecutive missed frames** (~0.5 seconds), the filter resets (`initialized = False`). This prevents the filter from extrapolating indefinitely on a stale velocity estimate, which would eventually diverge from reality.

**Why 15 frames?** This is a balance between:
- **Too few** (e.g., 3 frames): Frequent resets during minor turbidity → jumpy re-acquisition
- **Too many** (e.g., 60 frames): Prediction diverges significantly → large error when target reappears

At typical AUV speeds, 15 frames allows the vehicle to coast through a 0.5 s bubble cloud or momentary glare event without losing the tracking lock.

### 10.3 Growing Uncertainty During Prediction

During predict-only mode, the covariance $\mathbf{P}$ grows at each step:

$$\mathbf{P}_{k|k-1} = \mathbf{A} \mathbf{P}_{k-1|k-1} \mathbf{A}^T + \mathbf{Q}$$

Since no correction step shrinks $\mathbf{P}$, the position uncertainty **increases linearly** (approximately $\sigma_q \cdot \sqrt{n}$ after $n$ predict-only steps). When a detection finally arrives, the Kalman Gain $\mathbf{K}$ will be **larger than usual**, meaning the filter will trust the new measurement more heavily — this is correct behaviour, because our prediction has become increasingly uncertain.

---

## 11. Complete Closed-Loop Signal Flow

### 11.1 End-to-End Data Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        AUV VISUAL SERVOING LOOP                            │
│                                                                             │
│   ┌──────────┐   RTSP/UDP    ┌──────────┐   z_k        ┌──────────────┐   │
│   │  RPi 4B  │──────────────→│   YOLO   │────────────→│ KALMAN FILTER │   │
│   │  Camera  │   H.264       │  (GPU)   │  [u, v]     │              │   │
│   └──────────┘               └──────────┘  Raw px      │  predict()   │   │
│        ↑                                                │  correct()   │   │
│        │                                                │              │   │
│        │                                                │  State:      │   │
│        │                                                │  x̂=[x,y,    │   │
│        │                                                │     vx,vy]   │   │
│        │                                                └──────┬───────┘   │
│        │                                                       │           │
│        │                                               Smoothed [x̂, ŷ]    │
│        │                                                       │           │
│        │                                                       ▼           │
│        │                                              ┌────────────────┐   │
│        │                                              │  ERROR CALC    │   │
│        │                                              │  ex = (x̂-cx)  │   │
│        │                                              │       /(w/2)  │   │
│        │                                              │  ey = (ŷ-cy)  │   │
│        │                                              │       /(h/2)  │   │
│        │                                              └───────┬────────┘   │
│        │                                                      │            │
│        │                                              ┌───────▼────────┐   │
│        │                                              │ P-CONTROLLER   │   │
│        │                                              │ yaw = Kp·ex   │   │
│        │                                              │ heave = Kp·ey │   │
│        │                                              └───────┬────────┘   │
│        │                                                      │            │
│        │         ┌──────────┐  PWM     ┌──────────┐  MAVLink  │            │
│        │         │ 6 × ESC  │←─────────│ ArduSub  │←──────────┘            │
│        │         │ + Motors │          │ (Pixhawk)│  manual_control        │
│        │         └─────┬────┘          └──────────┘                        │
│        │               │                                                   │
│        │           Force τ                                                 │
│        │               ▼                                                   │
│        │      ┌────────────────────────────────────────┐                   │
│        │      │         AUV DYNAMICS                   │                   │
│        │      │  M·ν̇ + C(ν)·ν + D(ν)·ν + g(η) = τ    │                   │
│        │      │                                        │                   │
│        │      │  → Produces body velocity ν            │                   │
│        │      └────────────────┬───────────────────────┘                   │
│        │                       │                                           │
│        │                   ν = [u,v,w,p,q,r]                              │
│        │                       ▼                                           │
│        │      ┌────────────────────────────────────────┐                   │
│        │      │         AUV KINEMATICS                 │                   │
│        │      │  η̇ = J(η) · ν                         │                   │
│        │      │                                        │                   │
│        │      │  → Produces earth-frame pose η         │                   │
│        │      └────────────────┬───────────────────────┘                   │
│        │                       │                                           │
│        │                  New position                                     │
│        │                  & orientation                                    │
│        └───────────────────────┘                                           │
│              Camera viewpoint changes                                      │
│              → Target appears at new pixel location                        │
│              → Loop repeats at 30 Hz                                       │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 11.2 The Kalman Filter's Three Outputs and Their Effects

| KF Output | Value | Fed To | Effect on AUV Dynamics |
|-----------|-------|--------|----------------------|
| Smoothed position $[\hat{x}, \hat{y}]$ | Filtered pixel coords | Error calculation → P-controller | Smooth $\boldsymbol{\tau}$ → smooth $\dot{\boldsymbol{\nu}}$ → reduced inertial forces, less added-mass energy waste |
| Estimated velocity $[\hat{v}_x, \hat{v}_y]$ | px/frame | HUD display (future: predictive controller) | Enables anticipating target motion rather than purely reacting |
| Predicted position (occlusion) | Extrapolated coords | Error calculation (as fallback) | Maintains tracking through brief occlusions → continuous $\boldsymbol{\tau}$ → no abrupt stops/starts |

---

## 12. Numerical Example: 5-Cycle Walk-Through

We trace 5 consecutive execution cycles ($\Delta t = 0.03333$ s) of the 4D Visual Servoing Kalman Filter to observe jitter smoothing, velocity estimation, and occlusion bridging.

**Initial Conditions ($k = 0$)**:
$$\hat{\mathbf{x}}_{0|0} = \begin{bmatrix} 310.0 \\ 235.0 \\ 5.0 \\ -2.0 \end{bmatrix}, \quad \mathbf{P}_{0|0} = \begin{bmatrix} 1.0 & 0 & 0 & 0 \\ 0 & 1.0 & 0 & 0 \\ 0 & 0 & 1.0 & 0 \\ 0 & 0 & 0 & 1.0 \end{bmatrix}$$

### Cycle 1 ($k=1$): Raw Detection $\mathbf{z}_1 = [325.0, 233.0]$ (Jump of +15 px)

1. **Prediction Step**:
$$\hat{\mathbf{x}}_{1|0} = \mathbf{A}\hat{\mathbf{x}}_{0|0} = \begin{bmatrix} 310.1667 \\ 234.9333 \\ 5.0000 \\ -2.0000 \end{bmatrix}$$
$$\mathbf{P}_{1|0} = \mathbf{A}\mathbf{P}_{0|0}\mathbf{A}^T + \mathbf{Q} = \begin{bmatrix} 1.00111 & 0 & 0.03336 & 0 \\ 0 & 1.00111 & 0 & 0.03336 \\ 0.03336 & 0 & 1.00167 & 0 \\ 0 & 0.03336 & 0 & 1.00167 \end{bmatrix}$$

2. **Correction Step**:
- **Innovation**: $\tilde{\mathbf{y}}_1 = \mathbf{z}_1 - \mathbf{H}\hat{\mathbf{x}}_{1|0} = [+14.8333, -1.9333]^T$
- **Innovation Covariance**: $\mathbf{S}_1 = \mathbf{H}\mathbf{P}_{1|0}\mathbf{H}^T + \mathbf{R} = \text{diag}(1.20111, 1.20111)$
- **Kalman Gain**: $\mathbf{K}_1 = \mathbf{P}_{1|0}\mathbf{H}^T \mathbf{S}_1^{-1} = \begin{bmatrix} 0.83350 & 0 \\ 0 & 0.83350 \\ 0.02777 & 0 \\ 0 & 0.02777 \end{bmatrix}$
- **Updated State**:
$$\hat{\mathbf{x}}_{1|1} = \begin{bmatrix} 310.1667 \\ 234.9333 \\ 5.0000 \\ -2.0000 \end{bmatrix} + \mathbf{K}_1 \tilde{\mathbf{y}}_1 = \begin{bmatrix} 322.5242 \\ 233.3218 \\ 5.4119 \\ -2.0537 \end{bmatrix}$$

*The raw +15 px jump is smoothed to +12.35 px, suppressing thruster spikes while updating target velocity to $\hat{v}_x \approx 162.3$ px/s.*

### Cycle 2 ($k=2$): Raw Detection $\mathbf{z}_2 = [328.0, 231.0]$

The filter locks onto nominal tracking. The Kalman gain for position drops to $K_{pos} = 0.45511$.
- **Updated State**: $\hat{\mathbf{x}}_{2|2} = [325.1146, 232.2278, 5.9731, -2.2925]^T$

### Cycle 3 ($k=3$): Visual Occlusion (YOLO Detection Lost)

1. **Prediction Step**:
$$\hat{\mathbf{x}}_{3|2} = \mathbf{A}\hat{\mathbf{x}}_{2|2} = \begin{bmatrix} 325.3137 \\ 232.1514 \\ 5.9731 \\ -2.2925 \end{bmatrix}$$
2. **Occlusion Execution**: YOLO detects no bounding box. The system skips the correction step and retains the prediction: $\hat{\mathbf{x}}_{3|3} = \hat{\mathbf{x}}_{3|2}$ and $\mathbf{P}_{3|3} = \mathbf{P}_{3|2}$.
*The AUV continues tracking smooth predicted velocity trajectories without thruster command collapse.*

### Cycle 4 ($k=4$): Continued Occlusion (2nd Consecutive Missed Frame)

- **Prediction Step**: $\hat{\mathbf{x}}_{4|3} = [325.5128, 232.0750, 5.9731, -2.2925]^T$.
Correction is skipped again. The covariance $\mathbf{P}_{4|4}$ grows continuously by $\mathbf{Q}$.

### Cycle 5 ($k=5$): Target Re-Appears! Raw Detection $\mathbf{z}_5 = [332.0, 229.0]$

1. **Prediction Step**: $\hat{\mathbf{x}}_{5|4} = [325.7119, 231.9986, 5.9731, -2.2925]^T$
2. **Correction Step**: Due to accrued covariance during occlusion, the elevated Kalman Gain ($K_{pos} = 0.6214$) immediately locks back onto the re-observed target position without transient oscillations: $\hat{\mathbf{x}}_{5|5} = [330.12, 229.84, 6.42, -2.48]^T$.

---

## 13. Implementation Mapping to Code

### `kalman_filter.py` Implementation

```python
import cv2
import numpy as np

class AUVKalmanFilter:
    def __init__(self, dt=1.0/30.0, process_noise_std=0.05, measurement_noise_std=0.20):
        self.dt = dt
        # 1. State vector x = [x, y, vx, vy]^T (4 states, 2 measurements)
        self.kf = cv2.KalmanFilter(4, 2)
        
        # 2. State Transition Matrix A
        self.kf.transitionMatrix = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], np.float32)
        
        # 3. Measurement Observation Matrix H
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], np.float32)
        
        # 4. Discretized Process Noise Covariance Matrix Q (CWNA model)
        dt2 = (self.dt ** 2) / 2.0
        dt3 = (self.dt ** 3) / 3.0
        q_var = process_noise_std ** 2
        self.kf.processNoiseCov = q_var * np.array([
            [dt3, 0, dt2, 0],
            [0, dt3, 0, dt2],
            [dt2, 0, self.dt, 0],
            [0, dt2, 0, self.dt]
        ], np.float32)
        
        # 5. Measurement Noise Covariance Matrix R
        r_var = measurement_noise_std ** 2
        self.kf.measurementNoiseCov = r_var * np.eye(2, dtype=np.float32)
        
        # 6. Initial Estimation Error Covariance Matrix P
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)
        
        self.missed_frames = 0
        self.max_missed_frames = 15
```

### `auv_yolo_tracking.py` Integration

```python
# MAVLink Serial Command Dispatch to ArduSub Autopilot
mav.manual_control_send(
    target_system=1,
    x=0,          # Surge
    y=0,          # Sway
    z=pwm_heave,  # Heave
    r=pwm_yaw,    # Yaw
    buttons=0
)
```

---

## 14. Parameter Taxonomy Dictionary

| Variable / Symbol | Physical / Mathematical Definition | Value / Unit |
|---|---|---|
| $\mathbf{x}_k \in \mathbb{R}^4$ | State vector containing 2D position and velocity | $[x, y, v_x, v_y]^T$ (px, px/s) |
| $\mathbf{z}_k \in \mathbb{R}^2$ | Measurement vector containing raw YOLO centroid | $[u, v]^T$ (pixels) |
| $\mathbf{A} \in \mathbb{R}^{4 \times 4}$ | Constant velocity state transition matrix | Dimensionless ($\Delta t = 0.03333$ s) |
| $\mathbf{H} \in \mathbb{R}^{2 \times 4}$ | Measurement observation matrix | Extracting position entries |
| $\mathbf{Q} \in \mathbb{R}^{4 \times 4}$ | Process noise covariance matrix | CWNA model ($q_s = 0.05$) |
| $\mathbf{R} \in \mathbb{R}^{2 \times 2}$ | Measurement noise covariance matrix | Diagonal ($\sigma_r^2 = 0.20$) |
| $\mathbf{P}_{k|k-1} \in \mathbb{R}^{4 \times 4}$ | A priori estimation error covariance matrix | $\mathbb{E}[\boldsymbol{e}_{k|k-1}\boldsymbol{e}_{k|k-1}^T]$ |
| $\mathbf{P}_{k|k} \in \mathbb{R}^{4 \times 4}$ | A posteriori estimation error covariance matrix | Joseph form update |
| $\mathbf{K}_k \in \mathbb{R}^{4 \times 2}$ | Optimal Kalman Gain matrix | $\mathbf{P}_{k|k-1} \mathbf{H}^T \mathbf{S}_k^{-1}$ |
| $\mathbf{S}_k \in \mathbb{R}^{2 \times 2}$ | Innovation residual covariance matrix | $\mathbf{H} \mathbf{P} \mathbf{H}^T + \mathbf{R}$ |
| $c_x, c_y$ | Optical principal center coordinates | $(320, 240)$ pixels |
| $e_x, e_y$ | Normalized image plane error signals | $[-1.0, +1.0]$ dimensionless |
| $\tau_{yaw}, \tau_{heave}$ | ArduSub thruster manual control effort | $[-400, +400]$ PWM units |
| $\boldsymbol{p}^n \in \mathbb{R}^3$ | 3D position vector in Earth frame | $[x, y, z]^T$ (meters) |
| $\boldsymbol{q}^n \in \mathcal{S}^3$ | Unit quaternion orientation vector | $[q_0, q_1, q_2, q_3]^T$ |
| $\boldsymbol{v}^b \in \mathbb{R}^3$ | Linear body velocity vector | $[u, v, w]^T$ (m/s) |
| $\boldsymbol{b}_a, \boldsymbol{b}_g \in \mathbb{R}^3$ | Accelerometer and Gyroscope bias drift | m/s$^2$, rad/s |

---

## 15. Summary & Key Takeaways

### What the Kalman Filter Does for the AUV — In Three Sentences

> The Kalman Filter sits between the YOLO vision detector and the thruster controller. It uses a constant-velocity kinematic model to **smooth noisy pixel measurements** into a clean position-and-velocity state estimate, directly eliminating thruster jitter caused by detection noise. When the detector temporarily fails (occlusion), the filter **extrapolates the target's trajectory** using its last estimated velocity, allowing the AUV to continue tracking for up to 0.5 seconds without visual input.

### The Five Key Effects

| # | Effect | Mechanism | Benefit |
|---|--------|-----------|---------|
| 1 | **Thruster jitter elimination** | Smooths ±15 px noise to ±2 px | Extends motor life, reduces energy waste, eliminates acoustic disturbance |
| 2 | **Occlusion bridging** | Constant-velocity prediction for up to 15 frames | Maintains tracking through bubbles, turbidity, and glare |
| 3 | **Velocity estimation** | Infers $[v_x, v_y]$ from position-only measurements | Enables future predictive/feed-forward control |
| 4 | **Latency compensation** | Predict step propagates state forward in time | Partially compensates for camera-to-controller delay |
| 5 | **Dynamics suppression** | Filters $e_x, e_y$ | Suppresses destabilizing Munk Moment in decoupled 4-DOF plant |

---

## 16. References

1. **Fossen, T.I.** (2011). *Handbook of Marine Craft Hydrodynamics and Motion Control*. John Wiley & Sons. — Definitive reference for AUV kinematics and dynamics (6-DOF equations).

2. **Kalman, R.E.** (1960). "A New Approach to Linear Filtering and Prediction Problems". *Journal of Basic Engineering*, 82(1), 35–45. — The original Kalman Filter paper.

3. **Siciliano, B. & Khatib, O.** (Eds.) (2016). *Springer Handbook of Robotics*. Chapter 34: Visual Servoing. — IBVS and PBVS theory.

4. **Chaumette, F. & Hutchinson, S.** (2006). "Visual Servo Control Part I: Basic Approaches". *IEEE Robotics & Automation Magazine*, 13(4), 82–90.

5. **Welch, G. & Bishop, G.** (2006). "An Introduction to the Kalman Filter". UNC-Chapel Hill TR 95-041. — Widely cited Kalman Filter tutorial.

6. **OpenCV Documentation**. `cv2.KalmanFilter` class reference. https://docs.opencv.org/

7. **Ultralytics Documentation**. YOLO26 & YOLO-World Open-Vocabulary Architecture. https://docs.ultralytics.com/
