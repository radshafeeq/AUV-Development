# Comprehensive First-Principles Kinematic and Dynamic Modeling: Complete 6-DOF Formulation for ROV PIP (BlueROV2 Heavy Based) and Unabridged Variable-by-Variable 4-DOF Reduction for Poseidon AUV (BlueROV2 Standard Based)

> **Author**: Radhi Shafeeq  
> **Affiliation**: Hasanuddin University — Department of Mechatronics Engineering  
> **Undergraduate Thesis**: Design, Hydrodynamic Modeling, State Estimation, and Autonomous Visual Servoing for a 5-DOF Autonomous Underwater Vehicle (AUV)  
> **Vehicles Analyzed**: 
> 1. **Poseidon AUV**: Custom 4-DOF 6-motor AUV (BlueROV2 Standard frame design) operating under a decoupled 4-DOF reduction (Surge, Sway, Heave, Yaw) with passive metacentric stability in Roll and Pitch.
> 2. **ROV PIP**: 6-DOF 8-motor vehicle (BlueROV2 Heavy frame design) operating under active 6-DOF feedback control across all translational and rotational degrees of freedom.
>
> **Primary Academic Literature**:
> 1. Fossen, T. I. (2021). *Handbook of Marine Craft Hydrodynamics and Motion Control* (2nd ed.). John Wiley & Sons.
> 2. SNAME (1950). *Nomenclature for Treating the Motion of Vessels Through Fluids*. The Society of Naval Architects and Marine Engineers, Technical and Research Bulletin No. 1-5.
> 3. Kim, Y. V. (Ed.). (2023). *Kalman Filter - Engineering Applications*. IntechOpen. DOI: 10.5772/intechopen.100722.
> 4. Khalid, A., Sarwat, A., & Riggs, H. (Eds.). (2024). *Applications and Optimizations of Kalman Filter and Their Variants*. IntechOpen. DOI: 10.5772/intechopen.109154.
> 5. Särkkä, S., & Svensson, L. (2023). *Bayesian Filtering and Smoothing* (2nd ed.). Cambridge University Press. DOI: 10.1017/9781108910002.
>
> **LaTeX Formatting Notice**: All mathematical equations, scalars, vectors, matrices, and variables in this monograph are formatted with `$$...$$` delimiters for native, zero-error conversion into high-resolution rendered publication graphics in Google Docs using the **Auto-LaTeX Equations** add-on.

---

## Master Table of Contents

- [1. Executive Summary and Monograph Roadmap](#1-executive-summary-and-monograph-roadmap)
  - [1.1 Research Motivation and System Architectures](#11-research-motivation-and-system-architectures)
  - [1.2 Physical Hardware Comparison: Poseidon AUV vs. ROV PIP](#12-physical-hardware-comparison-poseidon-auv-vs-rov-pip)
- [2. Fundamental Mechanics and Coordinate Reference Frames](#2-fundamental-mechanics-and-coordinate-reference-frames)
  - [2.1 SNAME (1950) Conventions and Orthogonal Reference Frames](#21-sname-1950-conventions-and-orthogonal-reference-frames)
  - [2.2 Complete 6-DOF State Vectors and Nomenclature](#22-complete-6-dof-state-vectors-and-nomenclature)
  - [2.3 Ambient Hydrodynamic Flow, Ocean Current Modeling, and Relative Velocity](#23-ambient-hydrodynamic-flow-ocean-current-modeling-and-relative-velocity)
  - [2.4 Physics of Metacentric Righting Stability and Natural Frequencies](#24-physics-of-metacentric-righting-stability-and-natural-frequencies)
- [3. Exhaustive First-Principles Derivation of 6-DOF Kinematics](#3-exhaustive-first-principles-derivation-of-6-dof-kinematics)
  - [3.1 Step-by-Step Derivation of Linear Rotation Matrix R_b^n](#31-step-by-step-derivation-of-linear-rotation-matrix-r_bn)
  - [3.2 Derivation and Inversion of Angular Rate Transformation Matrix T_Theta](#32-derivation-and-inversion-of-angular-rate-transformation-matrix-t_theta)
  - [3.3 Singularities, Gimbal Lock, and Unit Quaternion Formulation](#33-singularities-gimbal-lock-and-unit-quaternion-formulation)
  - [3.4 Full 6x6 Kinematic Jacobian Matrix J(eta_2)](#34-full-6x6-kinematic-jacobian-matrix-jeta_2)
- [4. Exhaustive First-Principles Derivation of 6-DOF Dynamics (ROV PIP Plant)](#4-exhaustive-first-principles-derivation-of-6-dof-dynamics-rov-pip-plant)
  - [4.1 Total Mass Tensor M = M_RB + M_A](#41-total-mass-tensor-m--m_rb--m_a)
    - [4.1.1 Rigid-Body Mass Matrix M_RB](#411-rigid-body-mass-matrix-m_rb)
    - [4.1.2 Hydrodynamic Added Mass Matrix M_A](#412-hydrodynamic-added-mass-matrix-m_a)
    - [4.1.3 Combined System Mass Tensor M](#413-combined-system-mass-tensor-m)
  - [4.2 Coriolis and Centripetal Matrices](#42-coriolis-and-centripetal-matrices)
    - [4.2.1 Rigid-Body Coriolis Matrix C_RB(nu)](#421-rigid-body-coriolis-matrix-c_rbnu)
    - [4.2.2 Added Mass Coriolis Matrix C_A(nu_r)](#422-added-mass-coriolis-matrix-c_anu_r)
  - [4.3 Hydrodynamic Damping Tensor D(nu_r)](#43-hydrodynamic-damping-tensor-dnu_r)
    - [4.3.1 Linear Laminar Skin Friction D_L](#431-linear-laminar-skin-friction-d_l)
    - [4.3.2 Non-Linear Turbulent Quadratic Cross-Flow Drag D_NL(nu_r)](#432-non-linear-turbulent-quadratic-cross-flow-drag-d_nlnu_r)
  - [4.4 Full 6-DOF Hydrostatic Restoring Vector g(eta)](#44-full-6-dof-hydrostatic-restoring-vector-geta)
  - [4.5 Thruster Control Allocation for ROV PIP (8 Thrusters)](#45-thruster-control-allocation-for-rov-pip-8-thrusters)
    - [4.5.1 Thruster Dynamics & T200 Propeller Characteristics](#451-thruster-dynamics--t200-propeller-characteristics)
    - [4.5.2 6x8 Thruster Allocation Matrix T_6x8](#452-6x8-thruster-allocation-matrix-t_6x8)
    - [4.5.3 Moore-Penrose Pseudo-Inverse Allocation](#453-moore-penrose-pseudo-inverse-allocation)
- [5. Unabridged Variable-by-Variable Reduction from 6-DOF to Decoupled 4-DOF (Poseidon AUV)](#5-unabridged-variable-by-variable-reduction-from-6-dof-to-decoupled-4-dof-poseidon-auv)
  - [5.1 Physical Reduction Mechanism and Metacentric Spring Proof](#51-physical-reduction-mechanism-and-metacentric-spring-proof)
  - [5.2 Variable-by-Variable Kinematic Reduction](#52-variable-by-variable-kinematic-reduction)
  - [5.3 Variable-by-Variable Dynamic Reduction (Kinetics)](#53-variable-by-variable-dynamic-reduction-kinetics)
    - [5.3.1 Mass Tensor Reduction M -> M_4](#531-mass-tensor-reduction-m---m_4)
    - [5.3.2 Coriolis Tensor Reduction & The Hydrodynamic Munk Moment](#532-coriolis-tensor-reduction--the-hydrodynamic-munk-moment)
    - [5.3.3 Hydrodynamic Damping Tensor Reduction](#533-hydrodynamic-damping-tensor-reduction)
    - [5.3.4 Hydrostatic Restoring Vector Reduction g -> g_4](#534-hydrostatic-restoring-vector-reduction-g---g_4)
    - [5.3.5 Thruster Control Allocation for Poseidon AUV (6 Thrusters)](#535-thruster-control-allocation-for-poseidon-auv-6-thrusters)
  - [5.4 Unified Decoupled 4-DOF Non-linear Differential Equations](#54-unified-decoupled-4-dof-non-linear-differential-equations)
- [6. ArduSub Autopilot Architecture, SITL Mapping, and Motor Mixing](#6-ardusub-autopilot-architecture-sitl-mapping-and-motor-mixing)
  - [6.1 Hardware Flight Controller & Telemetry Interfacing](#61-hardware-flight-controller--telemetry-interfacing)
  - [6.2 Frame Parameter Configurations: -f vectored vs. -f vectored_6dof](#62-frame-parameter-configurations--f-vectored-vs--f-vectored_6dof)
  - [6.3 Gazebo Harmonic Physics System Integration](#63-gazebo-harmonic-physics-system-integration)
- [7. Master Comparative Synthesis: ROV PIP (6-DOF) vs. Poseidon AUV (4-DOF)](#7-master-comparative-synthesis-rov-pip-6-dof-vs-poseidon-auv-4-dof)
- [8. Hierarchical Variable-Within-Variable Parameter Taxonomy Dictionary](#8-hierarchical-variable-within-variable-parameter-taxonomy-dictionary)
- [9. Comprehensive Master Bibliography](#9-comprehensive-master-bibliography)

---

## 1. Executive Summary and Monograph Roadmap

### 1.1 Research Motivation and System Architectures

Operating in underwater marine environments presents unique mechanical, hydrodynamic, and control engineering challenges. Marine vehicles are governed by coupled non-linear hydrodynamics, unmodeled ocean current disturbances, and velocity-squared drag forces. 

To design high-performance model-based controllers, state observers, and Hardware-In-The-Loop (HIL) simulations, this monograph establishes the unabridged theoretical foundation for two robotic marine vehicles:

1. **Poseidon AUV (4-DOF 6-Motor Architecture)**:
   - Based on the BlueROV2 Standard frame design.
   - Actuated by 6x Blue Robotics T200 brushless thrusters (4 horizontal vectored at $$45^\circ$$ in the $$x_b-y_b$$ plane, plus 2 vertical heave thrusters).
   - Operates under a decoupled 4-DOF state-space reduction: **Surge ($$u$$)**, **Sway ($$v$$)**, **Heave ($$w$$)**, and **Yaw ($$r$$)**.
   - Roll ($$\phi$$) and Pitch ($$\theta$$) degrees of freedom are unactuated and held near level equilibrium by passive metacentric righting stiffness ($$GM_T = z_g - z_b > 0$$).

2. **ROV PIP (Full 6-DOF 8-Motor Architecture)**:
   - Based on the BlueROV2 Heavy retrofit frame design.
   - Actuated by 8x Blue Robotics T200 brushless thrusters (4 horizontal vectored at $$45^\circ$$ in the $$x_b-y_b$$ plane, plus 4 vertical corner thrusters arranged in dual-canted pairs).
   - Capable of full, simultaneous closed-loop feedback control across all 6 Degrees of Freedom: **Surge ($$u$$)**, **Sway ($$v$$)**, **Heave ($$w$$)**, **Roll ($$p$$)**, **Pitch ($$q$$)**, and **Yaw ($$r$$)**.
   - Allows steady-state pitch angle holding (`PITCH_HOLD`), enabling tilted optical inspection of seafloor pipes, seabed structures, and vertical ship hulls without vehicle translation.

---

### 1.2 Physical Hardware Comparison: Poseidon AUV vs. ROV PIP

| Engineering Property | Poseidon AUV (BlueROV2 Standard Frame) | ROV PIP (BlueROV2 Heavy Frame) |
|---|---|---|
| **Total Thruster Count** | 6x T200 Brushless Thrusters | 8x T200 Brushless Thrusters |
| **Horizontal Subsystem** | 4 thrusters vectored at $$45^\circ$$ in $$x_b-y_b$$ plane | 4 thrusters vectored at $$45^\circ$$ in $$x_b-y_b plane |
| **Vertical Subsystem** | 2 vertical thrusters mounted along transverse $$y_b$$ axis | 4 vertical corner thrusters mounted at chassis corners |
| **Actuated Degrees of Freedom** | 4 active DOFs: Surge, Sway, Heave, Yaw | 6 active DOFs: Surge, Sway, Heave, Roll, Pitch, Yaw |
| **Pitch Axis ($$\theta$$) Control** | Unactuated; passive metacentric self-righting | Active differential thrust torque control |
| **Roll Axis ($$\phi$$) Control** | Passive metacentric righting / equal thruster bias | Active differential thrust torque control |
| **ArduSub Frame Parameter** | `-f vectored` | `-f vectored_6dof` |
| **Dry Vehicle Mass ($$m$$)** | $$m \approx 11.5\text{ kg}$$ | $$m \approx 13.5 - 14.2\text{ kg}$$ (includes heavy frame retrofit) |
| **Displaced Water Volume ($$\nabla$$)** | $$\nabla \approx 0.0115\text{ m}^3$$ | $$\nabla \approx 0.0135 - 0.0142\text{ m}^3$$ |
| **Operational Control Matrix** | $$4 \times 6$$ Allocation Matrix $$\mathbf{T}_{4\times 6}$$ | $$6 \times 8$$ Allocation Matrix $$\mathbf{T}_{6\times 8}$$ |

---

## 2. Fundamental Mechanics and Coordinate Reference Frames

### 2.1 SNAME (1950) Conventions and Orthogonal Reference Frames

Spatial kinematics and kinetics are defined on the special Euclidean group $$SE(3) = SO(3) \ltimes \mathbb{R}^3$$, encompassing 3D translations and 3D rotations across two right-handed orthogonal Cartesian reference frames standardized by SNAME (1950) and Fossen (2021):

```
       North x_n
           ▲
           │          EARTH-FIXED FRAME {n} (NED)
           │
           │────────► East y_n
           │
           ▼ Down z_n (Gravity g points downward)
```

```
       Bow (Surge) x_b
           ▲
           │          BODY-FIXED FRAME {b}
           │
           │────────► Starboard (Sway) y_b
           │
           ▼ Keel (Heave) z_b (Normal down through chassis)
```

#### 1. Earth-Fixed Inertial Reference Frame $$\mathcal{F}^n = \{O_n, x_n, y_n, z_n\}$$
- **Origin $$O_n$$**: Fixed at an arbitrary geographic reference location on the sea surface.
- **Axis $$x_n$$**: Points true North along the local horizontal tangent plane to Earth's surface.
- **Axis $$y_n$$**: Points true East along the local horizontal tangent plane to Earth's surface.
- **Axis $$z_n$$**: Points vertically downward into the water column along local gravity $$\mathbf{g} = [0, 0, g]^T$$.

Because vehicle operational speeds ($$U < 2.0\text{ m/s}$$) and operational ranges ($$L < 5\text{ km}$$) are tiny compared to Earth's rotational speed ($$\Omega_E \approx 7.292 \times 10^{-5}\text{ rad/s}$$) and radius ($$R_E \approx 6.371 \times 10^6\text{ m}$$), the centrifugal acceleration ratio is:
$$\frac{a_{\text{centrifugal}}}{g} = \frac{\Omega_E^2 R_E}{g} \approx \frac{(7.292 \times 10^{-5})^2 (6.371 \times 10^6)}{9.81} \approx 0.0034 \ll 1$$
Therefore, $$\mathcal{F}^n$$ is treated as a true Newtonian inertial frame where Earth's rotational acceleration is neglected.

#### 2. Body-Fixed Moving Reference Frame $$\mathcal{F}^b = \{O_b, x_b, y_b, z_b\}$$
- **Origin $$O_b$$**: Located at the vehicle's geometric Center of Origin (CO), aligned with the center of the main acrylic pressure vessel.
- **Axis $$x_b$$**: Longitudinal axis pointing forward toward the bow (surge direction).
- **Axis $$y_b$$**: Transverse axis pointing starboard / right (sway direction).
- **Axis $$z_b$$**: Normal axis pointing vertically downward through the keel (heave direction).

---

### 2.2 Complete 6-DOF State Vectors and Nomenclature

The spatial motion of an underwater vehicle in 6 Degrees of Freedom (6-DOF) is defined by three primary vectors:

#### 1. Position and Orientation Vector in $$\mathcal{F}^n$$:
$$\boldsymbol{\eta} = \begin{bmatrix} \boldsymbol{\eta}_1 \\ \boldsymbol{\eta}_2 \end{bmatrix} = \begin{bmatrix} x \\ y \\ z \\ \phi \\ \theta \\ \psi \end{bmatrix} \in \mathbb{R}^6$$
- $$\boldsymbol{\eta}_1 = [x, y, z]^T \in \mathbb{R}^3$$: North, East, Down position coordinates (meters).
- $$\boldsymbol{\eta}_2 = [\phi, \theta, \psi]^T \in \mathbb{R}^3$$: Euler orientation angles: Roll ($$\phi$$), Pitch ($$\theta$$), Yaw ($$\psi$$) (radians).

#### 2. Linear and Angular Velocity Vector in $$\mathcal{F}^b$$:
$$\boldsymbol{\nu} = \begin{bmatrix} \boldsymbol{\nu}_1 \\ \boldsymbol{\nu}_2 \end{bmatrix} = \begin{bmatrix} u \\ v \\ w \\ p \\ q \\ r \end{bmatrix} \in \mathbb{R}^6$$
- $$\boldsymbol{\nu}_1 = [u, v, w]^T \in \mathbb{R}^3$$: Body-fixed linear velocities along Surge, Sway, and Heave (m/s).
- $$\boldsymbol{\nu}_2 = [p, q, r]^T \in \mathbb{R}^3$$: Body-fixed angular velocities about Roll, Pitch, and Yaw axes (rad/s).

#### 3. Generalized Forces and Moments Vector in $$\mathcal{F}^b$$:
$$\boldsymbol{\tau} = \begin{bmatrix} \boldsymbol{\tau}_1 \\ \boldsymbol{\tau}_2 \end{bmatrix} = \begin{bmatrix} X \\ Y \\ Z \\ K \\ M \\ N \end{bmatrix} \in \mathbb{R}^6$$
- $$\boldsymbol{\tau}_1 = [X, Y, Z]^T \in \mathbb{R}^3$$: Hydrodynamic forces along Surge, Sway, and Heave (Newtons).
- $$\boldsymbol{\tau}_2 = [K, M, N]^T \in \mathbb{R}^3$$: Hydrodynamic moments about Roll, Pitch, and Yaw axes ($$\text{N}\cdot\text{m}$$).

---

### 2.3 Ambient Hydrodynamic Flow, Ocean Current Modeling, and Relative Velocity

Let an irrotational, non-turbulent ocean current vector in $$\mathcal{F}^n$$ be denoted by:
$$\mathbf{V}_c^n = \begin{bmatrix} u_c^n & v_c^n & w_c^n & 0 & 0 & 0 \end{bmatrix}^T$$

Transforming the current into body coordinates via the transposed linear rotation matrix $$\mathbf{R}_n^b(\boldsymbol{\eta}_2)$$:
$$\boldsymbol{\nu}_c = \begin{bmatrix} \boldsymbol{\nu}_{c,1} \\ \boldsymbol{\nu}_{c,2} \end{bmatrix} = \begin{bmatrix} \mathbf{R}_n^b(\boldsymbol{\eta}_2) \mathbf{V}_{c,1}^n \\ \mathbf{0}_{3\times 1} \end{bmatrix} = \begin{bmatrix} u_c \\ v_c \\ w_c \\ 0 \\ 0 \\ 0 \end{bmatrix}$$

All hydrodynamic forces (added mass, damping, lift, and drag) depend strictly on the **relative velocity vector** $$\boldsymbol{\nu}_r \in \mathbb{R}^6$$:
$$\boldsymbol{\nu}_r = \boldsymbol{\nu} - \boldsymbol{\nu}_c = \begin{bmatrix} u - u_c \\ v - v_c \\ w - w_c \\ p \\ q \\ r \end{bmatrix} = \begin{bmatrix} u_r \\ v_r \\ w_r \\ p \\ q \\ r \end{bmatrix}$$

For a slowly varying current ($$\dot{\mathbf{V}}_c^n \approx \mathbf{0}$$), the time derivative of relative velocity in body coordinates is:
$$\dot{\boldsymbol{\nu}}_r = \dot{\boldsymbol{\nu}} - \dot{\boldsymbol{\nu}}_c = \dot{\boldsymbol{\nu}} + \begin{bmatrix} \mathbf{S}(\boldsymbol{\nu}_2) \boldsymbol{\nu}_{c,1} \\ \mathbf{0}_{3\times 1} \end{bmatrix}$$
where $$\mathbf{S}(\boldsymbol{\nu}_2)$$ is the skew-symmetric cross-product matrix of body angular velocities.

---

### 2.4 Physics of Metacentric Righting Stability and Natural Frequencies

Both vehicles are engineered with heavy ballast lead, battery pods, and aluminum housings at the bottom of the chassis, while lightweight syntactic foam buoyancy blocks sit at the top.
This establishes a physical vertical separation between the Center of Gravity $$\mathbf{r}_g = [x_g, y_g, z_g]^T$$ and Center of Buoyancy $$\mathbf{r}_b = [x_b, y_b, z_b]^T$$, creating a **positive metacentric height**:
$$GM_T = z_g - z_b > 0$$

When the vehicle tilts by roll angle $$\phi$$ or pitch angle $$\theta$$, downward gravity ($$W = mg$$) at CG and upward buoyancy ($$B = \rho g \nabla$$) at CB generate hydrostatic restoring righting moments:
$$K_{\text{restoring}}(\phi) = -(z_g W - z_b B) \sin\phi$$
$$M_{\text{restoring}}(\theta) = -(z_g W - z_b B) \sin\theta$$

Because $$(z_g W - z_b B) > 0$$, these restoring moments act as **torsional physical stiffness springs**:
$$k_\phi = z_g W - z_b B$$
$$k_\theta = z_g W - z_b B$$

Under small-angle approximations ($$\sin\phi \approx \phi, \sin\theta \approx \theta$$), the undamped natural frequencies in roll ($$\omega_{n,\phi}$$) and pitch ($$\omega_{n,\theta}$$) are:
$$\omega_{n,\phi} = \sqrt{\frac{z_g W - z_b B}{I_{xx} - K_{\dot{p}}}}$$
$$\omega_{n,\theta} = \sqrt{\frac{z_g W - z_b B}{I_{yy} - M_{\dot{q}}}}$$

where $$I_{xx} - K_{\dot{p}}$$ is total roll inertia (rigid body + added mass) and $$I_{yy} - M_{\dot{q}}$$ is total pitch inertia. These springs restore roll and pitch to horizontal equilibrium ($$\phi \to 0, \theta \to 0$$), naturally keeping $$p \approx 0$$ and $$q \approx 0$$.

---

## 3. Exhaustive First-Principles Derivation of 6-DOF Kinematics

Kinematics defines the geometric relationship mapping body velocities $$\boldsymbol{\nu}$$ to rates of change of world coordinates $$\dot{\boldsymbol{\eta}}$$:
$$\begin{bmatrix} \dot{\boldsymbol{\eta}}_1 \\ \dot{\boldsymbol{\eta}}_2 \end{bmatrix} = \begin{bmatrix} \mathbf{R}_b^n(\boldsymbol{\eta}_2) & \mathbf{0}_{3\times 3} \\ \mathbf{0}_{3\times 3} & \mathbf{T}_\Theta(\boldsymbol{\eta}_2) \end{bmatrix} \begin{bmatrix} \boldsymbol{\nu}_1 \\ \boldsymbol{\nu}_2 \end{bmatrix} \iff \dot{\boldsymbol{\eta}} = \mathbf{J}(\boldsymbol{\eta}_2) \boldsymbol{\nu}$$

---

### 3.1 Step-by-Step Derivation of Linear Rotation Matrix R_b^n

The rotation matrix $$\mathbf{R}_b^n(\boldsymbol{\eta}_2) \in SO(3)$$ transforms body linear velocities $$[u, v, w]^T$$ into Earth velocities $$[\dot{x}, \dot{y}, \dot{z}]^T$$ using the intrinsic $$z-y-x$$ (Yaw-Pitch-Roll) sequence.

#### Step 1: Principal Axis Rotations
1. **Yaw rotation ($$\psi$$) about $$z_n$$-axis**:
   $$\mathbf{R}_{z,\psi} = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
2. **Pitch rotation ($$\theta$$) about intermediate $$y'$$-axis**:
   $$\mathbf{R}_{y,\theta} = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$
3. **Roll rotation ($$\phi$$) about intermediate $$x''$$-axis**:
   $$\mathbf{R}_{x,\phi} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}$$

#### Step 2: Multiplication $$\mathbf{A} = \mathbf{R}_{y,\theta} \mathbf{R}_{x,\phi}$$
Evaluating matrix entries:
$$\mathbf{A} = \begin{bmatrix} \cos\theta & \sin\theta\sin\phi & \sin\theta\cos\phi \\ 0 & \cos\phi & -\sin\phi \\ -\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi \end{bmatrix}$$

#### Step 3: Complete Multiplication $$\mathbf{R}_b^n = \mathbf{R}_{z,\psi} \mathbf{A}$$
$$\mathbf{R}_b^n(\boldsymbol{\eta}_2) = \begin{bmatrix} 
\cos\psi\cos\theta & -\sin\psi\cos\phi + \cos\psi\sin\theta\sin\phi & \sin\psi\sin\phi + \cos\psi\sin\theta\cos\phi \\ 
\sin\psi\cos\theta & \cos\psi\cos\phi + \sin\psi\sin\theta\sin\phi & -\cos\psi\sin\phi + \sin\psi\sin\theta\cos\phi \\ 
-\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi 
\end{bmatrix}$$

#### Step 4: Inverse Matrix $$\mathbf{R}_n^b = (\mathbf{R}_b^n)^T$$
Because $$\mathbf{R}_b^n \in SO(3)$$, its inverse equals its transpose:
$$\mathbf{R}_n^b(\boldsymbol{\eta}_2) = \begin{bmatrix} 
\cos\psi\cos\theta & \sin\psi\cos\theta & -\sin\theta \\ 
-\sin\psi\cos\phi + \cos\psi\sin\theta\sin\phi & \cos\psi\cos\phi + \sin\psi\sin\theta\sin\phi & \cos\theta\sin\phi \\ 
\sin\psi\sin\phi + \cos\psi\sin\theta\cos\phi & -\cos\psi\sin\phi + \sin\psi\sin\theta\cos\phi & \cos\theta\cos\phi 
\end{bmatrix}$$

#### Step 5: Scalar Linear Kinematic Equations
$$\dot{x} = u(\cos\psi\cos\theta) + v(-\sin\psi\cos\phi + \cos\psi\sin\theta\sin\phi) + w(\sin\psi\sin\phi + \cos\psi\sin\theta\cos\phi)$$
$$\dot{y} = u(\sin\psi\cos\theta) + v(\cos\psi\cos\phi + \sin\psi\sin\theta\sin\phi) + w(-\cos\psi\sin\phi + \sin\psi\sin\theta\cos\phi)$$
$$\dot{z} = u(-\sin\theta) + v(\cos\theta\sin\phi) + w(\cos\theta\cos\phi)$$

---

### 3.2 Derivation and Inversion of Angular Rate Transformation Matrix T_Theta

Body angular rates $$[p, q, r]^T$$ represent projections of Euler angle rates $$[\dot{\phi}, \dot{\theta}, \dot{\psi}]^T$$ onto body axes:
$$\boldsymbol{\nu}_2 = \begin{bmatrix} p \\ q \\ r \end{bmatrix} = \begin{bmatrix} \dot{\phi} \\ 0 \\ 0 \end{bmatrix} + \mathbf{R}_{x,\phi}^T \begin{bmatrix} 0 \\ \dot{\theta} \\ 0 \end{bmatrix} + \mathbf{R}_{x,\phi}^T \mathbf{R}_{y,\theta}^T \begin{bmatrix} 0 \\ 0 \\ \dot{\psi} \end{bmatrix}$$

Evaluating transposed rotation projections:
$$\begin{bmatrix} p \\ q \\ r \end{bmatrix} = \begin{bmatrix} 1 & 0 & -\sin\theta \\ 0 & \cos\phi & \sin\phi\cos\theta \\ 0 & -\sin\phi & \cos\phi\cos\theta \end{bmatrix} \begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} \iff \boldsymbol{\nu}_2 = \mathbf{T}_\Theta^{-1}(\boldsymbol{\eta}_2)\dot{\boldsymbol{\eta}}_2$$

#### Inverting $$\mathbf{B} = \mathbf{T}_\Theta^{-1}$$ via Cofactor-Adjugate Method:
1. **Determinant**:
   $$\det(\mathbf{B}) = 1 \cdot (\cos^2\phi\cos\theta + \sin^2\phi\cos\theta) = \cos\theta$$
2. **Cofactor Matrix**:
   $$\text{Cof}(\mathbf{B}) = \begin{bmatrix} \cos\theta & 0 & 0 \\ \sin\phi\sin\theta & \cos\phi\cos\theta & \sin\phi \\ \cos\phi\sin\theta & -\sin\phi\cos\theta & \cos\phi \end{bmatrix}$$
3. **Adjugate Matrix $$\text{adj}(\mathbf{B}) = \text{Cof}(\mathbf{B})^T$$**:
   $$\text{adj}(\mathbf{B}) = \begin{bmatrix} \cos\theta & \sin\phi\sin\theta & \cos\phi\sin\theta \\ 0 & \cos\phi\cos\theta & -\sin\phi\cos\theta \\ 0 & \sin\phi & \cos\phi \end{bmatrix}$$
4. **Dividing by $$\det(\mathbf{B}) = \cos\theta$$**:
   $$\mathbf{T}_\Theta(\boldsymbol{\eta}_2) = \begin{bmatrix} 1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\ 0 & \cos\phi & -\sin\phi \\ 0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta} \end{bmatrix}$$

#### Scalar Angular Kinematic Equations:
$$\dot{\phi} = p + q(\sin\phi\tan\theta) + r(\cos\phi\tan\theta)$$
$$\dot{\theta} = q(\cos\phi) - r(\sin\phi)$$
$$\dot{\psi} = q\left(\frac{\sin\phi}{\cos\theta}\right) + r\left(\frac{\cos\phi}{\cos\theta}\right)$$

---

### 3.3 Singularities, Gimbal Lock, and Unit Quaternion Formulation

When pitch reaches $$\theta = \pm 90^\circ$$, $$\cos\theta = 0$$ and $$\tan\theta \to \pm\infty$$, causing a kinematic singularity (**Gimbal Lock**).
To avoid this in non-linear simulations, unit quaternions $$\mathbf{q} = [\eta, \epsilon_1, \epsilon_2, \epsilon_3]^T \in \mathcal{S}^3$$ satisfying $$\eta^2 + \epsilon_1^2 + \epsilon_2^2 + \epsilon_3^2 = 1$$ are used:
$$\begin{bmatrix} \dot{\eta} \\ \dot{\boldsymbol{\epsilon}} \end{bmatrix} = \frac{1}{2} \begin{bmatrix} -\boldsymbol{\epsilon}^T \\ \eta\mathbf{I}_{3\times 3} + \mathbf{S}(\boldsymbol{\epsilon}) \end{bmatrix} \boldsymbol{\nu}_2 = \frac{1}{2}\mathbf{E}(\mathbf{q})\boldsymbol{\nu}_2$$

The corresponding rotation matrix expressed in quaternions is:
$$\mathbf{R}(\mathbf{q}) = (\eta^2 - \boldsymbol{\epsilon}^T\boldsymbol{\epsilon})\mathbf{I}_{3\times 3} + 2\boldsymbol{\epsilon}\boldsymbol{\epsilon}^T + 2\eta\mathbf{S}(\boldsymbol{\epsilon})$$

---

### 3.4 Full 6x6 Kinematic Jacobian Matrix J(eta_2)

$$\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{z} \\ \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 
\cos\psi\cos\theta & -\sin\psi\cos\phi + \cos\psi\sin\theta\sin\phi & \sin\psi\sin\phi + \cos\psi\sin\theta\cos\phi & 0 & 0 & 0 \\ 
\sin\psi\cos\theta & \cos\psi\cos\phi + \sin\psi\sin\theta\sin\phi & -\cos\psi\sin\phi + \sin\psi\sin\theta\cos\phi & 0 & 0 & 0 \\ 
-\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi & 0 & 0 & 0 \\ 
0 & 0 & 0 & 1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\ 
0 & 0 & 0 & 0 & \cos\phi & -\sin\phi \\ 
0 & 0 & 0 & 0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta} 
\end{bmatrix} \begin{bmatrix} u \\ v \\ w \\ p \\ q \\ r \end{bmatrix}$$

---

## 4. Exhaustive First-Principles Derivation of 6-DOF Dynamics (ROV PIP Plant)

Fossen's full 6-DOF marine craft kinetics equation is formulated as:
$$\mathbf{M}\dot{\boldsymbol{\nu}} + \mathbf{C}_{RB}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{C}_A(\boldsymbol{\nu}_r)\boldsymbol{\nu}_r + \mathbf{D}(\boldsymbol{\nu}_r)\boldsymbol{\nu}_r + \mathbf{g}(\boldsymbol{\eta}) = \boldsymbol{\tau} + \boldsymbol{\tau}_{\text{ext}}$$

---

### 4.1 Total Mass Tensor M = M_RB + M_A

#### 4.1.1 Rigid-Body Mass Matrix M_RB
Formulated about Center of Origin $$O_b$$ with Center of Gravity offset $$\mathbf{r}_g = [x_g, y_g, z_g]^T$$:
$$\mathbf{M}_{RB} = \begin{bmatrix} m\mathbf{I}_{3\times 3} & -m\mathbf{S}(\mathbf{r}_g) \\ m\mathbf{S}(\mathbf{r}_g) & \mathbf{I}_g - m\mathbf{S}^2(\mathbf{r}_g) \end{bmatrix} \in \mathbb{R}^{6\times 6}$$

where $$m = 13.5\text{ kg}$$ is dry mass, and $$\mathbf{I}_g$$ is the rigid-body inertia tensor about CG:
$$\mathbf{I}_g = \begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz} \\ -I_{xy} & I_{yy} & -I_{yz} \\ -I_{xz} & -I_{yz} & I_{zz} \end{bmatrix} = \begin{bmatrix} 0.16 & 0 & 0 \\ 0 & 0.21 & 0 \\ 0 & 0 & 0.245 \end{bmatrix} \text{kg}\cdot\text{m}^2$$

Parallel-axis theorem term $$-m\mathbf{S}^2(\mathbf{r}_g)$$:
$$-\mathbf{S}^2(\mathbf{r}_g) = \begin{bmatrix} y_g^2 + z_g^2 & -x_g y_g & -x_g z_g \\ -x_g y_g & x_g^2 + z_g^2 & -y_g z_g \\ -x_g z_g & -y_g z_g & x_g^2 + y_g^2 \end{bmatrix}$$

When origin $$O_b$$ aligns with CG ($$\mathbf{r}_g = \mathbf{0}$$) and by structural symmetry:
$$\mathbf{M}_{RB} = \text{diag}[m, m, m, I_{xx}, I_{yy}, I_{zz}] = \text{diag}[13.5, 13.5, 13.5, 0.16, 0.21, 0.245]$$

#### 4.1.2 Hydrodynamic Added Mass Matrix M_A
Added mass models the inertia of fluid entrained and accelerated by the hull:
$$\mathbf{M}_A = -\begin{bmatrix} 
X_{\dot{u}} & X_{\dot{v}} & X_{\dot{w}} & X_{\dot{p}} & X_{\dot{q}} & X_{\dot{r}} \\ 
Y_{\dot{u}} & Y_{\dot{v}} & Y_{\dot{w}} & Y_{\dot{p}} & Y_{\dot{q}} & Y_{\dot{r}} \\ 
Z_{\dot{u}} & Z_{\dot{v}} & Z_{\dot{w}} & Z_{\dot{p}} & Z_{\dot{q}} & Z_{\dot{r}} \\ 
K_{\dot{u}} & K_{\dot{v}} & K_{\dot{w}} & K_{\dot{p}} & K_{\dot{q}} & K_{\dot{r}} \\ 
M_{\dot{u}} & M_{\dot{v}} & M_{\dot{w}} & M_{\dot{p}} & M_{\dot{q}} & M_{\dot{r}} \\ 
N_{\dot{u}} & N_{\dot{v}} & N_{\dot{w}} & N_{\dot{p}} & N_{\dot{q}} & N_{\dot{r}} 
\end{bmatrix}$$

For a symmetrical open-frame ROV at moderate speeds:
$$\mathbf{M}_A = -\text{diag}[X_{\dot{u}}, Y_{\dot{v}}, Z_{\dot{w}}, K_{\dot{p}}, M_{\dot{q}}, N_{\dot{r}}] = \text{diag}[6.36, 7.12, 18.68, 0.015, 0.080, 0.245]\text{ kg, kg}\cdot\text{m}^2$$

#### 4.1.3 Combined System Mass Tensor M
$$\mathbf{M} = \mathbf{M}_{RB} + \mathbf{M}_A = \text{diag}[m - X_{\dot{u}}, m - Y_{\dot{v}}, m - Z_{\dot{w}}, I_{xx} - K_{\dot{p}}, I_{yy} - M_{\dot{q}}, I_{zz} - N_{\dot{r}}]$$
$$\mathbf{M} = \text{diag}[19.86\text{ kg}, 20.62\text{ kg}, 32.18\text{ kg}, 0.175\text{ kg}\cdot\text{m}^2, 0.290\text{ kg}\cdot\text{m}^2, 0.490\text{ kg}\cdot\text{m}^2]$$

---

### 4.2 Coriolis and Centripetal Matrices

#### 4.2.1 Rigid-Body Coriolis Matrix C_RB(nu)
When $$\mathbf{r}_g = \mathbf{0}$$:
$$\mathbf{C}_{RB}(\boldsymbol{\nu}) = \begin{bmatrix} 
0 & 0 & 0 & 0 & mw & -mv \\ 
0 & 0 & 0 & -mw & 0 & mu \\ 
0 & 0 & 0 & mv & -mu & 0 \\ 
0 & mw & -mv & 0 & I_{zz}r & -I_{yy}q \\ 
-mw & 0 & mu & -I_{zz}r & 0 & I_{xx}p \\ 
mv & -mu & 0 & I_{yy}q & -I_{xx}p & 0 
\end{bmatrix}$$

#### 4.2.2 Added Mass Coriolis Matrix C_A(nu_r)
$$\mathbf{C}_A(\boldsymbol{\nu}_r) = \begin{bmatrix} 
0 & 0 & 0 & 0 & -Z_{\dot{w}}w_r & Y_{\dot{v}}v_r \\ 
0 & 0 & 0 & Z_{\dot{w}}w_r & 0 & -X_{\dot{u}}u_r \\ 
0 & 0 & 0 & -Y_{\dot{v}}v_r & X_{\dot{u}}u_r & 0 \\ 
0 & -Z_{\dot{w}}w_r & Y_{\dot{v}}v_r & 0 & -N_{\dot{r}}r & M_{\dot{q}}q \\ 
Z_{\dot{w}}w_r & 0 & -X_{\dot{u}}u_r & N_{\dot{r}}r & 0 & -K_{\dot{p}}p \\ 
-Y_{\dot{v}}v_r & X_{\dot{u}}u_r & 0 & -M_{\dot{q}}q & K_{\dot{p}}p & 0 
\end{bmatrix}$$

---

### 4.3 Hydrodynamic Damping Tensor D(nu_r)

Hydrodynamic damping combines linear skin friction and quadratic cross-flow drag:
$$\mathbf{D}(\boldsymbol{\nu}_r) = \mathbf{D}_L + \mathbf{D}_{NL}(\boldsymbol{\nu}_r)$$
$$\mathbf{D}_L = -\text{diag}[X_u, Y_v, Z_w, K_p, M_q, N_r]$$
$$\mathbf{D}_{NL}(\boldsymbol{\nu}_r) = -\text{diag}[X_{u|u|}|u_r|, Y_{v|v|}|v_r|, Z_{w|w|}|w_r|, K_{p|p|}|p|, M_{q|q|}|q|, N_{r|r|}|r|]$$

Multiplying by relative velocity $$\boldsymbol{\nu}_r$$:
$$\mathbf{D}(\boldsymbol{\nu}_r)\boldsymbol{\nu}_r = \begin{bmatrix} 
-(X_u + X_{u|u|}|u_r|)u_r \\ 
-(Y_v + Y_{v|v|}|v_r|)v_r \\ 
-(Z_w + Z_{w|w|}|w_r|)w_r \\ 
-(K_p + K_{p|p|}|p|)p \\ 
-(M_q + M_{q|q|}|q|)q \\ 
-(N_r + N_{r|r|}|r|)r 
\end{bmatrix}$$

---

### 4.4 Full 6-DOF Hydrostatic Restoring Vector g(eta)

Hydrostatic forces combine gravitational weight $$W = mg$$ acting downward at CG and buoyant force $$B = \rho g \nabla$$ acting upward at CB:
$$\mathbf{f}_g^b = \mathbf{R}_n^b \begin{bmatrix} 0 \\ 0 \\ W \end{bmatrix} = \begin{bmatrix} -W\sin\theta \\ W\cos\theta\sin\phi \\ W\cos\theta\cos\phi \end{bmatrix}, \qquad \mathbf{f}_b^b = \mathbf{R}_n^b \begin{bmatrix} 0 \\ 0 \\ -B \end{bmatrix} = \begin{bmatrix} B\sin\theta \\ -B\cos\theta\sin\phi \\ -B\cos\theta\cos\phi \end{bmatrix}$$

Evaluating restoring moments $$\boldsymbol{\tau}_g = \mathbf{r}_g \times \mathbf{f}_g^b$$ and $$\boldsymbol{\tau}_b = \mathbf{r}_b \times \mathbf{f}_b^b$$:
$$\mathbf{g}(\boldsymbol{\eta}) = \begin{bmatrix} 
(W - B)\sin\theta \\ 
-(W - B)\cos\theta\sin\phi \\ 
-(W - B)\cos\theta\cos\phi \\ 
-(y_g W - y_b B)\cos\theta\cos\phi + (z_g W - z_b B)\cos\theta\sin\phi \\ 
(z_g W - z_b B)\sin\theta + (x_g W - x_b B)\cos\theta\cos\phi \\ 
-(x_g W - x_b B)\cos\theta\sin\phi - (y_g W - y_b B)\sin\theta 
\end{bmatrix}$$

When CO aligns with CB ($$\mathbf{r}_b = \mathbf{0}$$) and CG is at $$[0, 0, z_g]^T$$ ($$z_g > 0$$):
$$\mathbf{g}(\boldsymbol{\eta}) = \begin{bmatrix} 
(W - B)\sin\theta \\ 
-(W - B)\cos\theta\sin\phi \\ 
-(W - B)\cos\theta\cos\phi \\ 
z_g W\cos\theta\sin\phi \\ 
z_g W\sin\theta \\ 
0 
\end{bmatrix}$$

---

### 4.5 Thruster Control Allocation for ROV PIP (8 Thrusters)

#### 4.5.1 T200 Propeller Characteristics
Each T200 thruster generates thrust according to:
$$T_i = K_T \rho D_p^4 n_i |n_i|$$
where $$D_p = 0.076\text{ m}$$, $$\rho = 1025\text{ kg/m}^3$$, and $$K_T \approx 0.11$$.

#### 4.5.2 6x8 Thruster Allocation Matrix T_6x8
The $$6 \times 8$$ Thruster Allocation Matrix $$\mathbf{T}_{6\times 8}$$ maps individual thruster forces $$\mathbf{f} \in \mathbb{R}^8$$ to 6-DOF body generalized forces/moments $$\boldsymbol{\tau} \in \mathbb{R}^6$$:
$$\boldsymbol{\tau} = \mathbf{T}_{6\times 8} \mathbf{f}$$
where each column $$\mathbf{t}_i$$ represents the unit force and torque arm of thruster $$i$$:
$$\mathbf{t}_i = \begin{bmatrix} \mathbf{d}_i \\ \mathbf{r}_i \times \mathbf{d}_i \end{bmatrix} \in \mathbb{R}^6$$
$$\mathbf{d}_i$$ is the unit direction vector of thrust, and $$\mathbf{r}_i$$ is the position vector of thruster $$i$$ relative to origin $$O_b$$.

#### 4.5.3 Moore-Penrose Pseudo-Inverse Allocation
Because the 8-thruster Heavy system is over-actuated ($$8 > 6$$), the control allocation problem:
$$\min_{\mathbf{f}} \frac{1}{2} \mathbf{f}^T \mathbf{W} \mathbf{f} \quad \text{subject to} \quad \boldsymbol{\tau} = \mathbf{T}_{6\times 8} \mathbf{f}$$
has the unique closed-form solution via the Moore-Penrose pseudo-inverse:
$$\mathbf{f} = \mathbf{T}_{6\times 8}^\dagger \boldsymbol{\tau} = \mathbf{T}_{6\times 8}^T (\mathbf{T}_{6\times 8} \mathbf{T}_{6\times 8}^T)^{-1} \boldsymbol{\tau}$$

---

## 5. Unabridged Variable-by-Variable Reduction from 6-DOF to Decoupled 4-DOF (Poseidon AUV)

### 5.1 Physical Reduction Mechanism and Metacentric Spring Proof

Positive metacentric height ($$GM_T = z_g - z_b \approx 0.020\text{ m} > 0$$) creates physical restoring stiffness springs:
$$k_\phi = z_g W, \quad k_\theta = z_g W$$
Under normal operating speeds, these springs constrain the vehicle to near-zero roll and pitch:
$$\phi \to 0, \quad \theta \to 0, \quad p \to 0, \quad q \to 0, \quad \dot{p} \to 0, \quad \dot{q} \to 0$$

---

### 5.2 Variable-by-Variable Kinematic Reduction

#### 1. State Vector Reduction
$$\boldsymbol{\eta}_{\text{6-DOF}} = \begin{bmatrix} x \\ y \\ z \\ \phi \\ \theta \\ \psi \end{bmatrix} \xrightarrow{\phi=0,\theta=0} \boldsymbol{\eta}_{\text{4-DOF}} = \begin{bmatrix} x \\ y \\ z \\ \psi \end{bmatrix} \in \mathbb{R}^4$$
$$\boldsymbol{\nu}_{\text{6-DOF}} = \begin{bmatrix} u \\ v \\ w \\ p \\ q \\ r \end{bmatrix} \xrightarrow{p=0,q=0} \boldsymbol{\nu}_{\text{4-DOF}} = \begin{bmatrix} u \\ v \\ w \\ r \end{bmatrix} \in \mathbb{R}^4$$

#### 2. Linear Rotation Matrix Reduction $$\mathbf{R}_b^n \to \mathbf{R}_4$$
Evaluating $$\mathbf{R}_b^n(\boldsymbol{\eta}_2)$$ at $$\phi = 0$$ and $$\theta = 0$$:
$$\mathbf{R}_b^n \Big|_{\phi=0,\theta=0} = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

#### 3. Angular Transformation Matrix Reduction $$\mathbf{T}_\Theta \to \mathbf{I}_{3\times 3}$$
$$\mathbf{T}_\Theta \Big|_{\phi=0,\theta=0} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} \implies \dot{\psi} = r$$

#### 4. Reduced Kinematic Jacobian $$\mathbf{J}_4(\psi)$$
$$\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{z} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 
\cos\psi & -\sin\psi & 0 & 0 \\ 
\sin\psi & \cos\psi & 0 & 0 \\ 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1 
\end{bmatrix} \begin{bmatrix} u \\ v \\ w \\ r \end{bmatrix} \iff \dot{\boldsymbol{\eta}}_4 = \mathbf{J}_4(\psi)\boldsymbol{\nu}_4$$

---

### 5.3 Variable-by-Variable Dynamic Reduction (Kinetics)

#### 5.3.1 Mass Tensor Reduction $$\mathbf{M} \to \mathbf{M}_4$$
Deleting rows 4-5 and columns 4-5:
$$\mathbf{M}_{\text{4-DOF}} = \begin{bmatrix} 
m - X_{\dot{u}} & 0 & 0 & 0 \\ 
0 & m - Y_{\dot{v}} & 0 & 0 \\ 
0 & 0 & m - Z_{\dot{w}} & 0 \\ 
0 & 0 & 0 & I_{zz} - N_{\dot{r}} 
\end{bmatrix} = \begin{bmatrix} 
17.86 & 0 & 0 & 0 \\ 
0 & 18.62 & 0 & 0 \\ 
0 & 0 & 30.18 & 0 \\ 
0 & 0 & 0 & 0.250 
\end{bmatrix}$$

#### 5.3.2 Coriolis Tensor Reduction & The Hydrodynamic Munk Moment
Multiplying the full Coriolis matrix $$(\mathbf{C}_{RB} + \mathbf{C}_A)$$ by $$\boldsymbol{\nu}_r = [u_r, v_r, w_r, 0, 0, r]^T$$:
1. **Surge Row 1**:
   $$\tau_{X,\text{Coriolis}} = -(m - Y_{\dot{v}}) v_r r$$
2. **Sway Row 2**:
   $$\tau_{Y,\text{Coriolis}} = (m - X_{\dot{u}}) u_r r$$
3. **Heave Row 3**:
   $$\tau_{Z,\text{Coriolis}} = 0$$
4. **Yaw Row 6**:
   $$\tau_{N,\text{Coriolis}} = -m v_r u_r + Y_{\dot{v}} v_r u_r + m u_r v_r - X_{\dot{u}} u_r v_r = (X_{\dot{u}} - Y_{\dot{v}}) u_r v_r$$

Substituting numerical added mass derivatives:
$$X_{\dot{u}} - Y_{\dot{v}} = -6.36 - (-7.12) = +0.76 > 0$$

> **The Destabilizing Munk Moment**: Because transverse sway added mass ($$|Y_{\dot{v}}| = 7.12\text{ kg}$$) exceeds longitudinal surge added mass ($$|X_{\dot{u}}| = 6.36\text{ kg}$$), the Munk torque gain is positive ($$+0.76\text{ kg}$$). During forward cruise ($$u_r > 0$$) with slight cross-flow ($$v_r > 0$$), this torque turns the vehicle broadside to the flow, requiring active yaw feedback control ($$N$$) to prevent spinning!

#### 5.3.3 Hydrodynamic Damping Tensor Reduction
- Surge Damping: $$-(X_u + X_{u|u|}|u_r|)u_r$$
- Sway Damping: $$-(Y_v + Y_{v|v|}|v_r|)v_r$$
- Heave Damping: $$-(Z_w + Z_{w|w|}|w_r|)w_r$$
- Yaw Damping: $$-(N_r + N_{r|r|}|r|)r$$

#### 5.3.4 Hydrostatic Restoring Vector Reduction $$\mathbf{g} \to \mathbf{g}_4$$
Evaluating at $$\phi = 0, \theta = 0$$:
$$\mathbf{g}_4 = \begin{bmatrix} 0 \\ 0 \\ \rho g \nabla - mg \\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ B - W \\ 0 \end{bmatrix}$$

#### 5.3.5 Thruster Control Allocation for Poseidon AUV (6 Thrusters)
Setting unactuated moments $$\tau_K = 0$$ and $$\tau_M = 0$$, the control matrix reduces to the $$4 \times 6$$ matrix $$\mathbf{T}_{4\times 6}$$:
$$\mathbf{f}_{\text{thruster}} = \mathbf{T}_{4\times 6}^\dagger \boldsymbol{\tau} = \mathbf{T}_{4\times 6}^T (\mathbf{T}_{4\times 6} \mathbf{T}_{4\times 6}^T)^{-1} \boldsymbol{\tau} \in \mathbb{R}^6$$

---

### 5.4 Unified Decoupled 4-DOF Non-linear Differential Equations

Combining all variable-by-variable reductions yields the complete 4-DOF plant governing the Poseidon AUV:

$$\text{\textbf{Surge:}} \quad (m - X_{\dot{u}})\dot{u} - (m - Y_{\dot{v}})v_r r - (X_u + X_{u|u|}|u_r|)u_r = \tau_X$$

$$\text{\textbf{Sway:}} \quad (m - Y_{\dot{v}})\dot{v} + (m - X_{\dot{u}})u_r r - (Y_v + Y_{v|v|}|v_r|)v_r = \tau_Y$$

$$\text{\textbf{Heave:}} \quad (m - Z_{\dot{w}})\dot{w} - (Z_w + Z_{w|w|}|w_r|)w_r + (\rho g \nabla - mg) = \tau_Z$$

$$\text{\textbf{Yaw:}} \quad (I_{zz} - N_{\dot{r}})\dot{r} + (X_{\dot{u}} - Y_{\dot{v}})u_r v_r - (N_r + N_{r|r|}|r|)r = \tau_N$$

---

## 6. ArduSub Autopilot Architecture, SITL Mapping, and Motor Mixing

### 6.1 Hardware Flight Controller & Telemetry Interfacing
- **Flight Controller**: Pixhawk 2.4.8 running ArduSub firmware.
- **Companion Computer**: Raspberry Pi 4B running BlueOS 1.4.5, bridging telemetry over Ethernet (`192.168.2.2`).
- **Telemetry Protocols**: MAVLink 2.0 streaming `NAMED_VALUE_FLOAT`, `ATTITUDE`, `LOCAL_POSITION_NED`, and `MANUAL_CONTROL`.

### 6.2 Frame Parameter Configurations: -f vectored vs. -f vectored_6dof
- **Poseidon AUV (`-f vectored`)**: 6 thruster channels. Mixer maps 4 DOFs (Surge, Sway, Heave, Yaw). Vertical thrusters are biased equally to maintain level trim.
- **ROV PIP (`-f vectored_6dof`)**: 8 thruster channels. Mixer maps all 6 DOFs, enabling pitch-angle target tracking (`PITCH_HOLD`).

### 6.3 Gazebo Harmonic Physics System Integration
SITL models define:
- `gz::sim::systems::Buoyancy` calculating displaced volume $$\nabla$$ and fluid density $$\rho = 1000\text{ kg/m}^3$$.
- `gz::sim::systems::Hydrodynamics` simulating added mass $$\mathbf{M}_A$$ and damping tensors $$\mathbf{D}_L, \mathbf{D}_{NL}$$.
- `gz::sim::systems::OdometryPublisher` streaming ground-truth 6-DOF velocity telemetry at 50 Hz.

---

## 7. Master Comparative Synthesis: ROV PIP (6-DOF) vs. Poseidon AUV (4-DOF)

| Technical Metric | ROV PIP (6-DOF Active Plant) | Poseidon AUV (Decoupled 4-DOF Reduction) |
|---|---|---|
| **Thruster Count** | 8 T200 Brushless Thrusters | 6 T200 Brushless Thrusters |
| **Horizontal Layout** | 4 Vectored at $$45^\circ$$ in $$x_b-y_b$$ plane | 4 Vectored at $$45^\circ$$ in $$x_b-y_b$$ plane |
| **Vertical Layout** | 4 Vertical corner thrusters (dual-canted pairs) | 2 Side-by-side vertical thrusters along $$y_b$$ axis |
| **ArduSub Frame Setting** | `-f vectored_6dof` | `-f vectored` |
| **Active Controllable DOFs** | Full 6-DOF (Surge, Sway, Heave, Roll, Pitch, Yaw) | 4-DOF (Surge, Sway, Heave, Yaw) |
| **Pitch Axis ($$\theta$$) Authority** | Active differential thrust torque authority | Unactuated; passive metacentric righting spring |
| **Roll Axis ($$\phi$$) Authority** | Active differential thrust torque authority | Passive righting / equalized vertical thruster bias |
| **Allocation Matrix Size** | $$6 \times 8$$ matrix ($$\text{rank} = 6$$) | $$4 \times 6$$ matrix ($$\text{rank} = 4$$) |
| **State Vector Dimension** | $$\boldsymbol{\eta} \in \mathbb{R}^6, \, \boldsymbol{\nu} \in \mathbb{R}^6$$ | $$\boldsymbol{\eta}_4 \in \mathbb{R}^4, \, \boldsymbol{\nu}_4 \in \mathbb{R}^4$$ |
| **Cross-Coupling Dynamics** | Full $$6 \times 6$$ non-linear Coriolis coupling | Destabilizing Munk Moment $$(X_{\dot{u}} - Y_{\dot{v}})u_r v_r$$ in Yaw |
| **Inspection Capability** | Tilted pitch inspection (nose-down seabed survey) | Strictly horizontal orientation; fixed pitch angle |

---

## 8. Hierarchical Variable-Within-Variable Parameter Taxonomy Dictionary

| Variable / Sub-Variable | Physical Definition and Derivation Source | Nominal Value / Unit |
|---|---|---|
| $$m$$ | Dry physical mass of vehicle ($$m = \int_V \rho_{\text{body}} dV$$) | $$11.5\text{ kg (Poseidon)}, 13.5\text{ kg (PIP)}$$ |
| $$\rho$$ | Fluid mass density of seawater | $$1025\text{ kg/m}^3$$ |
| $$\nabla$$ | Displaced volume of water ($$\nabla = \int_\nabla dV$$) | $$0.0115\text{ m}^3\text{ (Poseidon)}, 0.0134\text{ m}^3\text{ (PIP)}$$ |
| $$g$$ | Acceleration due to gravity constant | $$9.81\text{ m/s}^2$$ |
| $$W$$ | Gravitational weight force ($$W = mg$$) | $$112.82\text{ N (Poseidon)}, 132.44\text{ N (PIP)}$$ |
| $$B$$ | Hydrostatic buoyant force ($$B = \rho g \nabla$$) | $$115.12\text{ N (Poseidon)}, 134.74\text{ N (PIP)}$$ |
| $$B - W$$ | Net vertical positive buoyancy force | $$+2.30\text{ N}$$ |
| $$\mathbf{r}_g = [x_g, y_g, z_g]^T$$ | CG position offset relative to origin $$O_b$$ | $$[0, 0, 0.02]^T\text{ m}$$ |
| $$\mathbf{r}_b = [x_b, y_b, z_b]^T$$ | CB position offset relative to origin $$O_b$$ | $$[0, 0, 0.00]^T\text{ m}$$ |
| $$z_g - z_b$$ | Metacentric height separating CB and CG | $$0.020\text{ m}$$ |
| $$I_{xx}, I_{yy}, I_{zz}$$ | Rigid moments of inertia ($$I_{zz} = \int (x^2 + y^2)dm$$) | $$0.16, 0.21, 0.245\text{ kg}\cdot\text{m}^2$$ |
| $$X_{\dot{u}}, Y_{\dot{v}}, Z_{\dot{w}}$$ | Hydrodynamic added mass ($$M_{A,ij} = -\rho \iint \phi_i \frac{\partial\phi_j}{\partial n} dS$$) | $$-6.36, -7.12, -18.68\text{ kg}$$ |
| $$K_{\dot{p}}, M_{\dot{q}}, N_{\dot{r}}$$ | Hydrodynamic added rotational inertia | $$-0.015, -0.080, -0.245\text{ kg}\cdot\text{m}^2$$ |
| $$m_u, m_v, m_w$$ | Virtual total masses ($$m_u = m - X_{\dot{u}}$$) | $$17.86, 18.62, 30.18\text{ kg}$$ |
| $$I_r$$ | Virtual total yaw inertia ($$I_r = I_{zz} - N_{\dot{r}}$$) | $$0.250\text{ kg}\cdot\text{m}^2\text{ (Poseidon)}, 0.490\text{ kg}\cdot\text{m}^2\text{ (PIP)}$$ |
| $$X_{\dot{u}} - Y_{\dot{v}}$$ | Munk Moment cross-coupling yaw torque gain | $$+0.76\text{ kg}$$ |
| $$X_u, Y_v, Z_w$$ | Linear skin friction damping derivatives | $$-13.7, 0.0, -33.8\text{ Ns/m}$$ |
| $$N_r$$ | Linear yaw angular friction derivative | $$-0.50\text{ Ns}\cdot\text{m/rad}$$ |
| $$X_{u|u|}, Y_{v|v|}, Z_{w|w|}$$ | Quadratic drag coefficients ($$\frac{1}{2}\rho C_d A_{\text{proj}}$$) | $$-141.0, -217.0, -190.0\text{ Ns}^2/\text{m}^2$$ |
| $$N_{r|r|}$$ | Quadratic yaw moment drag coefficient | $$-1.55\text{ Ns}^2/\text{rad}^2$$ |
| $$D_p$$ | T200 propeller diameter | $$0.076\text{ m}$$ |
| $$K_T(J_a)$$ | Dimensionless thrust coefficient | $$0.11$$ |
| $$\mathbf{T}_{6\times 8}^\dagger$$ | ROV PIP 8-thruster pseudo-inverse allocation matrix | $$8 \times 6\text{ matrix}$$ |
| $$\mathbf{T}_{4\times 6}^\dagger$$ | Poseidon AUV 6-thruster pseudo-inverse allocation matrix | $$6 \times 4\text{ matrix}$$ |

---

## 9. Comprehensive Master Bibliography

1. **Fossen, T. I.** (2021). *Handbook of Marine Craft Hydrodynamics and Motion Control* (2nd ed.). John Wiley & Sons. — Chapters 2–8: 6-DOF kinematics, rigid body dynamics, added mass, damping, restoring forces, and thruster allocation.
2. **SNAME** (1950). *Nomenclature for Treating the Motion of Vessels Through Fluids*. The Society of Naval Architects and Marine Engineers, Technical and Research Bulletin No. 1-5. — Foundational standard for underwater vehicle kinematics and hydrodynamic notation.
3. **Kim, Y. V.** (Ed.). (2023). *Kalman Filter - Engineering Applications*. IntechOpen. ISBN: 978-1-80356-575-0, DOI: 10.5772/intechopen.100722. — High-speed discrete filtering in autonomous robotic navigation.
4. **Khalid, A., Sarwat, A., & Riggs, H.** (Eds.). (2024). *Applications and Optimizations of Kalman Filter and Their Variants*. IntechOpen. ISBN: 978-0-85466-565-5. — Advanced non-linear estimation, disturbance observer design, and real-time marine vehicle stabilization.
5. **Särkkä, S., & Svensson, L.** (2023). *Bayesian Filtering and Smoothing* (2nd ed.). Cambridge University Press. DOI: 10.1017/9781108910002. — Continuous-discrete state estimation and Lyapunov differential propagation.
6. **Blue Robotics**. (2024). *BlueROV2 Technical Manual and Thruster Specifications*. Blue Robotics Inc., Torrance, CA.
7. **Shafeeq, R.** (2026). *AUV Development Repository: Autonomous Target Tracking and Visual Servoing Control*. GitHub Repository: [radshafeeq/AUV-Development](https://github.com/radshafeeq/AUV-Development).
