# Comprehensive First-Principles Kinematic and Dynamic Modeling, Complete 6-DOF Derivation for BlueROV2 Heavy, and Variable-by-Variable 4-DOF Reduction for BlueROV2 Standard

> **Author**: Radhi Shafeeq  
> **Context**: Undergraduate Mechatronics Engineering Thesis — Hasanuddin University  
> **System**: BlueROV2 Heavy (8-Thruster Active 6-DOF) and BlueROV2 Standard (6-Thruster Decoupled 4-DOF Reduction)  
> **Grounded In**: Master AUV Research Library, Gemini Notebook Theoretical Monographs, and Repository Archives  

---

## Abstract

This master technical monograph provides an unabridged, mathematically rigorous first-principles derivation of the six-degree-of-freedom (6-DOF) kinematic and dynamic equations of motion for an autonomous underwater vehicle (AUV), parameterized specifically for the **BlueROV2 Heavy** (8-thruster active 6-DOF vehicle) and the **BlueROV2 Standard** (6-thruster vehicle operating under a decoupled 4-DOF reduction). 

Beginning with the Society of Naval Architects and Marine Engineers (SNAME 1950) coordinate conventions, we derive the $SO(3)$ linear rotation matrix $\mathbf{R}_b^n(\boldsymbol{\eta}_2)$ and the angular velocity transformation matrix $\mathbf{T}_\Theta(\boldsymbol{\eta}_2)$ step-by-step using elementary principal axis rotations and cofactor-adjugate matrix inversion. We detail pitch gimbal lock singularities and formulate unit quaternion kinematics $\dot{\mathbf{q}} = \frac{1}{2}\mathbf{E}(\mathbf{q})\boldsymbol{\nu}_2$. We then establish Fossen's 6-DOF non-linear kinetics plant model from Newton-Euler momentum conservation and potential flow hydrodynamics, expanding every constituent tensor: rigid-body mass $\mathbf{M}_{RB}$, hydrodynamic added mass $\mathbf{M}_A$, rigid-body Coriolis $\mathbf{C}_{RB}(\boldsymbol{\nu})$, added mass Coriolis $\mathbf{C}_A(\boldsymbol{\nu}_r)$, ITTC-1957 skin friction and Morison quadratic drag $\mathbf{D}(\boldsymbol{\nu}_r)$, hydrostatic restoring vector $\mathbf{g}(\boldsymbol{\eta})$, and Moore-Penrose pseudo-inverse thruster control allocation $\boldsymbol{\tau} = \mathbf{T}\mathbf{f}$.

Subsequently, we prove how positive metacentric height ($GM_T = z_g - z_b > 0$) acts as a physical torsional spring that passively stabilizes roll and pitch ($\phi \approx 0, \theta \approx 0, p \approx 0, q \approx 0$). We execute a variable-by-variable reduction step-by-step, showing how every state variable, sub-variable, matrix row/column, tensor product, and hydrostatic component reduces from 6-DOF to a decoupled 4-DOF non-linear state-space model (Surge, Sway, Heave, Yaw), explicitly proving non-linear cross-coupling phenomena such as the destabilizing hydrodynamic Munk Moment $(X_{\dot{u}} - Y_{\dot{v}})u_r v_r$. We detail the software architecture in ArduSub, SITL frame mappings (`-f vectored` vs. `-f vectored_6dof`), and Gazebo physics integration. Finally, a master comparative synthesis and a hierarchical parameter taxonomy dictionary define every scalar, vector, tensor, and physical parameter.

---

## Table of Contents

1. [Executive Summary and Monograph Roadmap](#1-executive-summary-and-monograph-roadmap)
2. [Fundamental Mechanics, Coordinate Frames, and Physical Hardware Architectures](#2-fundamental-mechanics-coordinate-frames-and-physical-hardware-architectures)
3. [Exhaustive First-Principles Derivation of 6-DOF Kinematics](#3-exhaustive-first-principles-derivation-of-6-dof-kinematics)
4. [Exhaustive First-Principles Derivation of 6-DOF Dynamics (BlueROV2 Heavy Plant)](#4-exhaustive-first-principles-derivation-of-6-dof-dynamics-bluerov2-heavy-plant)
5. [Unabridged Variable-by-Variable Reduction from 6-DOF to Decoupled 4-DOF (BlueROV2 Standard)](#5-unabridged-variable-by-variable-reduction-from-6-dof-to-decoupled-4-dof-bluerov2-standard)
6. [ArduSub Software Architecture, SITL Frame Mapping, and Control Architecture](#6-ardusub-software-architecture-sitl-frame-mapping-and-control-architecture)
7. [Comparative Architectural Synthesis: BlueROV2 Heavy (6-DOF) vs. BlueROV2 Standard (4-DOF)](#7-comparative-architectural-synthesis-bluerov2-heavy-6-dof-vs-bluerov2-standard-4-dof)
8. [Hierarchical Variable-Within-Variable Parameter Taxonomy Dictionary](#8-hierarchical-variable-within-variable-parameter-taxonomy-dictionary)
9. [References](#9-references)

---

## 1. Executive Summary and Monograph Roadmap

### 1.1 Research Motivation and Technical Scope

Underwater robotic platforms operating in subsea environments encounter complex non-linear hydrodynamic forces, fluid-structure interactions, and multi-body coupling. To design robust feedback control loops, state estimators, and Hardware-In-The-Loop (HITL) or Software-In-The-Loop (SITL) simulations, a complete, unabridged mathematical model of the vehicle's kinematics and dynamics is mandatory.

This monograph serves as an exhaustive, publication-grade academic reference detailing the complete mathematical mechanics of two primary underwater vehicle configurations produced by Blue Robotics:

1. **The BlueROV2 Heavy**: An 8-thruster hovering remotely operated vehicle / autonomous underwater vehicle (HAUV) capable of active control across all 6 Degrees of Freedom (6-DOF): surge, sway, heave, roll, pitch, and yaw.
2. **The BlueROV2 Standard**: A 6-thruster configuration operating under a decoupled 4-DOF state-space reduction (surge, sway, heave, yaw), where roll and pitch are held near horizontal equilibrium by passive metacentric restoring stiffness ($GM_T = z_g - z_b > 0$).

Specifically, control system design for trajectory tracking, dynamic positioning (DP), and fault-tolerant thruster allocation requires a clear distinction between:
- Active, actuated force/moment degrees of freedom driven by propeller thrust allocation matrices.
- Unactuated degrees of freedom stabilized by hydrostatic buoyancy-gravity moment arms.
- Hydrodynamic cross-coupling terms (such as Coriolis forces and the destabilizing Munk moment) that induce parasitic yaw and pitch torques during translational maneuvers.

---

## 2. Fundamental Mechanics, Coordinate Frames, and Physical Hardware Architectures

### 2.1 SNAME (1950) Conventions and Orthogonal Reference Frames

Following the convention established by the Society of Naval Architects and Marine Engineers (SNAME 1950) and formalized by Fossen (2021), spatial motion is formulated in the special Euclidean group $SE(3) = SO(3) \ltimes \mathbb{R}^3$, representing 3D translations and 3D rotations across two right-handed orthogonal Cartesian reference frames.

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

#### 2.1.1 Earth-Fixed Inertial Reference Frame $\mathcal{F}^n$
The Earth-Fixed Reference Frame $\mathcal{F}^n = \{O_n, x_n, y_n, z_n\}$ uses a North-East-Down (NED) sequence:
- **Origin $O_n$**: Fixed at an arbitrary reference point on the calm water surface.
- **Axis $x_n$**: Points true North along the local horizontal tangent plane to Earth's surface.
- **Axis $y_n$**: Points true East along the local horizontal tangent plane to Earth's surface.
- **Axis $z_n$**: Points vertically downward into the water column along local gravity $\mathbf{g} = [0, 0, g]^T$.

Because subsea vehicle speeds ($U < 2.0\,\text{m/s}$) and operational ranges ($L < 10\,\text{km}$) are small relative to Earth's rotational velocity ($\Omega_E \approx 7.292 \times 10^{-5}\,\text{rad/s}$) and curvature radius ($R_E \approx 6371\,\text{km}$), the ratio of centrifugal acceleration to gravitational acceleration satisfies:

$$\frac{a_{\text{centrifugal}}}{g} = \frac{\Omega_E^2 R_E}{g} \approx \frac{(7.292 \times 10^{-5})^2 (6.371 \times 10^6)}{9.81} \approx 0.0034 \ll 1$$

Thus, $\mathcal{F}^n$ is treated as an absolute Newtonian inertial reference frame where Coriolis and centripetal accelerations caused by planetary rotation are identically zero.

#### 2.1.2 Body-Fixed Moving Reference Frame $\mathcal{F}^b$
The Body-Fixed Moving Reference Frame $\mathcal{F}^b = \{O_b, x_b, y_b, z_b\}$ is rigidly attached to the vehicle hull and moves relative to $\mathcal{F}^n$:
- **Origin $O_b$**: Located at the Center of Origin (CO), chosen to coincide with the Center of Gravity (CG) or the geometric center of the main acrylic pressure hull.
- **Axis $x_b$**: Longitudinal axis pointing forward along the symmetry plane of the hull (surge direction).
- **Axis $y_b$**: Transverse axis pointing to starboard / right (sway direction).
- **Axis $z_b$**: Normal axis pointing vertically downward through the bottom chassis (heave direction).

---

### 2.2 Complete 6-DOF State Vectors and SNAME Nomenclature

The spatial motion of a marine craft in 6 Degrees of Freedom (6-DOF) is defined by three primary vectors:

1. **Position and Orientation Vector in $\mathcal{F}^n$**:
   $$\boldsymbol{\eta} = \begin{bmatrix} \boldsymbol{\eta}_1 \\ \boldsymbol{\eta}_2 \end{bmatrix} = \begin{bmatrix} x \\ y \\ z \\ \phi \\ \theta \\ \psi \end{bmatrix} \in \mathbb{R}^6$$
   where $\boldsymbol{\eta}_1 = [x, y, z]^T \in \mathbb{R}^3$ represents position coordinates (North, East, Down in meters), and $\boldsymbol{\eta}_2 = [\phi, \theta, \psi]^T \in \mathbb{R}^3$ represents Euler orientation angles (roll, pitch, yaw in radians).

2. **Linear and Angular Velocity Vector in $\mathcal{F}^b$**:
   $$\boldsymbol{\nu} = \begin{bmatrix} \boldsymbol{\nu}_1 \\ \boldsymbol{\nu}_2 \end{bmatrix} = \begin{bmatrix} u \\ v \\ w \\ p \\ q \\ r \end{bmatrix} \in \mathbb{R}^6$$
   where $\boldsymbol{\nu}_1 = [u, v, w]^T \in \mathbb{R}^3$ represents body-fixed linear velocities along surge, sway, and heave ($\text{m/s}$), and $\boldsymbol{\nu}_2 = [p, q, r]^T \in \mathbb{R}^3$ represents body-fixed angular velocities about $x_b, y_b, z_b$ axes ($\text{rad/s}$).

3. **Generalized Forces and Moments Vector in $\mathcal{F}^b$**:
   $$\boldsymbol{\tau} = \begin{bmatrix} \boldsymbol{\tau}_1 \\ \boldsymbol{\tau}_2 \end{bmatrix} = \begin{bmatrix} X \\ Y \\ Z \\ K \\ M \\ N \end{bmatrix} \in \mathbb{R}^6$$
   where $\boldsymbol{\tau}_1 = [X, Y, Z]^T \in \mathbb{R}^3$ represents body-fixed forces along surge, sway, and heave ($\text{N}$), and $\boldsymbol{\tau}_2 = [K, M, N]^T \in \mathbb{R}^3$ represents body-fixed moments about roll, pitch, and yaw ($\text{N}\cdot\text{m}$).

---

### 2.3 Ambient Hydrodynamic Flow, Ocean Current Modeling, and Relative Velocity

In subsea operations, ambient fluid velocity (ocean currents) significantly alters the hydrodynamic forces acting on the vehicle. Let $\mathbf{V}_c^n = [u_c^n, v_c^n, w_c^n, 0, 0, 0]^T$ represent an irrotational, non-turbulent fluid current vector defined in $\mathcal{F}^n$.

The current velocity vector is transformed into the body frame $\mathcal{F}^b$ via the transposed linear rotation matrix $\mathbf{R}_n^b(\boldsymbol{\eta}_2)$:

$$\boldsymbol{\nu}_c = \begin{bmatrix} \boldsymbol{\nu}_{c,1} \\ \boldsymbol{\nu}_{c,2} \end{bmatrix} = \begin{bmatrix} \mathbf{R}_n^b(\boldsymbol{\eta}_2)\mathbf{V}_{c,1}^n \\ \mathbf{0}_{3\times 1} \end{bmatrix} = \begin{bmatrix} u_c \\ v_c \\ w_c \\ 0 \\ 0 \\ 0 \end{bmatrix}$$

All hydrodynamic forces (added mass, damping, lift, and drag) depend strictly on the **relative velocity vector** $\boldsymbol{\nu}_r \in \mathbb{R}^6$:

$$\boldsymbol{\nu}_r = \boldsymbol{\nu} - \boldsymbol{\nu}_c = \begin{bmatrix} u - u_c \\ v - v_c \\ w - w_c \\ p \\ q \\ r \end{bmatrix} = \begin{bmatrix} u_r \\ v_r \\ w_r \\ p \\ q \\ r \end{bmatrix}$$

For a constant or slowly varying current in $\mathcal{F}^n$ ($\dot{\mathbf{V}}_c^n = \mathbf{0}$), the time derivative of relative velocity in body coordinates is:

$$\dot{\boldsymbol{\nu}}_r = \dot{\boldsymbol{\nu}} - \dot{\boldsymbol{\nu}}_c = \dot{\boldsymbol{\nu}} - \begin{bmatrix} \dot{\mathbf{R}}_n^b \mathbf{V}_{c,1}^n \\ \mathbf{0}_{3\times 1} \end{bmatrix} = \dot{\boldsymbol{\nu}} + \begin{bmatrix} \mathbf{S}(\boldsymbol{\nu}_2)\boldsymbol{\nu}_{c,1} \\ \mathbf{0}_{3\times 1} \end{bmatrix}$$

where $\mathbf{S}(\boldsymbol{\nu}_2)$ is the skew-symmetric cross-product matrix operator of body angular rates.

---

### 2.4 Detailed Hardware Architecture Comparison: BlueROV2 Standard vs. BlueROV2 Heavy

| Architectural Component | BlueROV2 Standard (4-DOF Reduction) | BlueROV2 Heavy (Full 6-DOF Plant) |
|---|---|---|
| **Thruster Count** | 6x Blue Robotics T200 Brushless Thrusters | 8x Blue Robotics T200 Brushless Thrusters |
| **Horizontal Vectored Subsystem** | 4 thrusters vectored at $45^\circ$ angles in $x_b-y_b$ plane | 4 thrusters vectored at $45^\circ$ angles in $x_b-y_b$ plane |
| **Vertical Subsystem** | 2 vertical thrusters mounted side-by-side along $y_b$ axis | 4 vertical thrusters mounted at outer corners of chassis |
| **Actuated Degrees of Freedom** | 4 active DOFs: Surge ($X$), Sway ($Y$), Heave ($Z$), Yaw ($N$) | 6 active DOFs: Surge, Sway, Heave, Roll ($K$), Pitch ($M$), Yaw |
| **Pitch Axis ($\theta$) Stability** | Unactuated; stabilized passively by metacentric stiffness | Actively stabilized and controlled by vertical thruster pairs |
| **Roll Axis ($\phi$) Stability** | Passively stabilized by metacentric stiffness + equal thruster bias | Actively stabilized and controlled by vertical thruster pairs |
| **ArduSub Frame Parameter** | `-f vectored` (Mixer matrix size $4 \times 6$) | `-f vectored_6dof` (Mixer matrix size $6 \times 8$) |
| **Dry Vehicle Mass** | $m \approx 10.0 - 11.5\,\text{kg}$ | $m \approx 13.0 - 14.2\,\text{kg}$ (includes retrofit aluminum frame & ballast) |

---

### 2.5 Physics of Metacentric Righting Stability and Natural Frequencies

The physical layout of both BlueROV2 variants features heavy ballast lead blocks, heavy battery pods, and aluminum electronics enclosures mounted at the bottom of the frame, while lightweight syntactic foam buoyancy blocks are mounted at the top. This design creates a vertical separation between the Center of Gravity $\mathbf{r}_g = [x_g, y_g, z_g]^T$ and the Center of Buoyancy $\mathbf{r}_b = [x_b, y_b, z_b]^T$, establishing a positive metacentric height:

$$GM_T = z_g - z_b > 0$$

where $z_g > 0$ and $z_b = 0$ when origin $O_b$ coincides with the Center of Buoyancy (CB).

When tilted by a roll angle $\phi$ or pitch angle $\theta$, gravitational weight ($W = mg$) acting downward at CG and buoyancy ($B = \rho g \nabla$) acting upward at CB generate righting hydrostatic restoring moments:

$$K_{\text{restoring}}(\phi) = -(z_g W - z_b B) \sin\phi, \qquad M_{\text{restoring}}(\theta) = -(z_g W - z_b B) \sin\theta$$

Because $(z_g W - z_b B) > 0$, these moments act as physical torsional stiffness springs:

$$k_\phi = z_g W, \qquad k_\theta = z_g W$$

Under small-angle approximations ($\sin\phi \approx \phi, \sin\theta \approx \theta$), the undamped natural frequencies of the vehicle in roll ($\omega_{n,\phi}$) and pitch ($\omega_{n,\theta}$) are:

$$\omega_{n,\phi} = \sqrt{\frac{z_g W - z_b B}{I_{xx} - K_{\dot{p}}}}, \qquad \omega_{n,\theta} = \sqrt{\frac{z_g W - z_b B}{I_{yy} - M_{\dot{q}}}}$$

where $I_{xx} - K_{\dot{p}}$ is the total roll inertia (rigid body + added mass) and $I_{yy} - M_{\dot{q}}$ is the total pitch inertia. These restoring springs passively restore roll and pitch to horizontal equilibrium ($\phi \to 0, \theta \to 0$), holding angular rates $p \approx 0$ and $q \approx 0$.

---

## 3. Exhaustive First-Principles Derivation of 6-DOF Kinematics

Kinematics defines the geometric relationship mapping body velocities $\boldsymbol{\nu} = [\boldsymbol{\nu}_1^T, \boldsymbol{\nu}_2^T]^T$ to rates of change of global coordinates $\dot{\boldsymbol{\eta}} = [\dot{\boldsymbol{\eta}}_1^T, \dot{\boldsymbol{\eta}}_2^T]^T$:

$$\begin{bmatrix} \dot{\boldsymbol{\eta}}_1 \\ \dot{\boldsymbol{\eta}}_2 \end{bmatrix} = \begin{bmatrix} \mathbf{R}_b^n(\boldsymbol{\eta}_2) & \mathbf{0}_{3\times 3} \\ \mathbf{0}_{3\times 3} & \mathbf{T}_\Theta(\boldsymbol{\eta}_2) \end{bmatrix} \begin{bmatrix} \boldsymbol{\nu}_1 \\ \boldsymbol{\nu}_2 \end{bmatrix} \iff \dot{\boldsymbol{\eta}} = \mathbf{J}(\boldsymbol{\eta}_2) \boldsymbol{\nu}$$

---

### 3.1 Step-by-Step Derivation of Linear Rotation Matrix $\mathbf{R}_b^n$

The rotation matrix $\mathbf{R}_b^n(\boldsymbol{\eta}_2) \in SO(3)$ maps body linear velocities $[u, v, w]^T$ to global velocities $[\dot{x}, \dot{y}, \dot{z}]^T$. It is derived from three elementary right-handed principal axis rotations using the $z$-$y$-$x$ (yaw-pitch-roll) intrinsic sequence.

#### Step 1: Elementary Principal Axis Rotations

1. **Yaw rotation ($\psi$) about $z_n$-axis**:
   $$\mathbf{R}_{z,\psi} = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

2. **Pitch rotation ($\theta$) about intermediate $y'$-axis**:
   $$\mathbf{R}_{y,\theta} = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$

3. **Roll rotation ($\phi$) about intermediate $x''$-axis**:
   $$\mathbf{R}_{x,\phi} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}$$

#### Step 2: Explicit Matrix Multiplication $\mathbf{A} = \mathbf{R}_y \mathbf{R}_x$
We evaluate $\mathbf{A} = \mathbf{R}_{y,\theta} \mathbf{R}_{x,\phi}$ cell-by-cell using $A_{ij} = \sum_{k=1}^3 (\mathbf{R}_{y,\theta})_{ik} (\mathbf{R}_{x,\phi})_{kj}$:
- $A_{11} = (\cos\theta)(1) + (0)(0) + (\sin\theta)(0) = \cos\theta$
- $A_{12} = (\cos\theta)(0) + (0)(\cos\phi) + (\sin\theta)(\sin\phi) = \sin\theta\sin\phi$
- $A_{13} = (\cos\theta)(0) + (0)(-\sin\phi) + (\sin\theta)(\cos\phi) = \sin\theta\cos\phi$
- $A_{21} = (0)(1) + (1)(0) + (0)(0) = 0$
- $A_{22} = (0)(0) + (1)(\cos\phi) + (0)(\sin\phi) = \cos\phi$
- $A_{23} = (0)(0) + (1)(-\sin\phi) + (0)(\cos\phi) = -\sin\phi$
- $A_{31} = (-\sin\theta)(1) + (0)(0) + (\cos\theta)(0) = -\sin\theta$
- $A_{32} = (-\sin\theta)(0) + (0)(\cos\phi) + (\cos\theta)(\sin\phi) = \cos\theta\sin\phi$
- $A_{33} = (-\sin\theta)(0) + (0)(-\sin\phi) + (\cos\theta)(\cos\phi) = \cos\theta\cos\phi$

$$\mathbf{A} = \begin{bmatrix} \cos\theta & \sin\theta\sin\phi & \sin\theta\cos\phi \\ 0 & \cos\phi & -\sin\phi \\ -\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi \end{bmatrix}$$

#### Step 3: Explicit Matrix Multiplication $\mathbf{R}_b^n = \mathbf{R}_z \mathbf{A}$
Multiplying $\mathbf{R}_{z,\psi} \mathbf{A}$ gives the complete linear rotation matrix:

$$\mathbf{R}_b^n(\boldsymbol{\eta}_2) = \begin{bmatrix} 
\cos\psi\cos\theta & -\sin\psi\cos\phi + \cos\psi\sin\theta\sin\phi & \sin\psi\sin\phi + \cos\psi\sin\theta\cos\phi \\ 
\sin\psi\cos\theta & \cos\psi\cos\phi + \sin\psi\sin\theta\sin\phi & -\cos\psi\sin\phi + \sin\psi\sin\theta\cos\phi \\ 
-\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi 
\end{bmatrix}$$

#### Step 4: Transpose and Inverse Matrix $\mathbf{R}_n^b$
Because $\mathbf{R}_b^n \in SO(3)$, its inverse equals its transpose: $(\mathbf{R}_b^n)^{-1} = (\mathbf{R}_b^n)^T = \mathbf{R}_n^b$:

$$\mathbf{R}_n^b(\boldsymbol{\eta}_2) = \begin{bmatrix} 
\cos\psi\cos\theta & \sin\psi\cos\theta & -\sin\theta \\ 
-\sin\psi\cos\phi + \cos\psi\sin\theta\sin\phi & \cos\psi\cos\phi + \sin\psi\sin\theta\sin\phi & \cos\theta\sin\phi \\ 
\sin\psi\sin\phi + \cos\psi\sin\theta\cos\phi & -\cos\psi\sin\phi + \sin\psi\sin\theta\cos\phi & \cos\theta\cos\phi 
\end{bmatrix}$$

Orthogonality holds: $\mathbf{R}_b^n \mathbf{R}_n^b = \mathbf{I}_{3\times 3}$.

#### Step 5: Scalar Linear Kinematic Equations
Expanding $\dot{\boldsymbol{\eta}}_1 = \mathbf{R}_b^n(\boldsymbol{\eta}_2)\boldsymbol{\nu}_1$:

$$\dot{x} = u(\cos\psi\cos\theta) + v(-\sin\psi\cos\phi + \cos\psi\sin\theta\sin\phi) + w(\sin\psi\sin\phi + \cos\psi\sin\theta\cos\phi)$$
$$\dot{y} = u(\sin\psi\cos\theta) + v(\cos\psi\cos\phi + \sin\psi\sin\theta\sin\phi) + w(-\cos\psi\sin\phi + \sin\psi\sin\theta\cos\phi)$$
$$\dot{z} = u(-\sin\theta) + v(\cos\theta\sin\phi) + w(\cos\theta\cos\phi)$$

---

### 3.2 Derivation and Inversion of Angular Rate Transformation Matrix $\mathbf{T}_\Theta$

Body angular rates $[p, q, r]^T$ measured by the onboard IMU represent projections of Euler angle rates $[\dot{\phi}, \dot{\theta}, \dot{\psi}]^T$ onto body-fixed axes:

$$\boldsymbol{\nu}_2 = \begin{bmatrix} p \\ q \\ r \end{bmatrix} = \begin{bmatrix} \dot{\phi} \\ 0 \\ 0 \end{bmatrix} + \mathbf{R}_{x,\phi}^T \begin{bmatrix} 0 \\ \dot{\theta} \\ 0 \end{bmatrix} + \mathbf{R}_{x,\phi}^T \mathbf{R}_{y,\theta}^T \begin{bmatrix} 0 \\ 0 \\ \dot{\psi} \end{bmatrix}$$

Evaluating transposed rotation projections step-by-step:
1. Pitch rate projection:
   $$\mathbf{R}_{x,\phi}^T \begin{bmatrix} 0 \\ \dot{\theta} \\ 0 \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & \sin\phi \\ 0 & -\sin\phi & \cos\phi \end{bmatrix} \begin{bmatrix} 0 \\ \dot{\theta} \\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\ \dot{\theta}\cos\phi \\ -\dot{\theta}\sin\phi \end{bmatrix}$$

2. Yaw rate projection through pitch matrix:
   $$\mathbf{R}_{y,\theta}^T \begin{bmatrix} 0 \\ 0 \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} \cos\theta & 0 & -\sin\theta \\ 0 & 1 & 0 \\ \sin\theta & 0 & \cos\theta \end{bmatrix} \begin{bmatrix} 0 \\ 0 \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} -\dot{\psi}\sin\theta \\ 0 \\ \dot{\psi}\cos\theta \end{bmatrix}$$

3. Yaw rate projection through roll matrix:
   $$\mathbf{R}_{x,\phi}^T \begin{bmatrix} -\dot{\psi}\sin\theta \\ 0 \\ \dot{\psi}\cos\theta \end{bmatrix} = \begin{bmatrix} -\dot{\psi}\sin\theta \\ \dot{\psi}\sin\phi\cos\theta \\ \dot{\psi}\cos\phi\cos\theta \end{bmatrix}$$

Summing these vectors yields the direct mapping matrix $\mathbf{T}_\Theta^{-1}(\boldsymbol{\eta}_2)$:

$$\begin{bmatrix} p \\ q \\ r \end{bmatrix} = \begin{bmatrix} 1 & 0 & -\sin\theta \\ 0 & \cos\phi & \sin\phi\cos\theta \\ 0 & -\sin\phi & \cos\phi\cos\theta \end{bmatrix} \begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} \iff \boldsymbol{\nu}_2 = \mathbf{T}_\Theta^{-1}(\boldsymbol{\eta}_2)\dot{\boldsymbol{\eta}}_2$$

#### Inverting $\mathbf{B} = \mathbf{T}_\Theta^{-1}$ via Cofactor-Adjugate Method:
1. **Determinant $\det(\mathbf{B})$**:
   $$\det(\mathbf{B}) = 1 \cdot (\cos^2\phi\cos\theta + \sin^2\phi\cos\theta) - 0 + (-\sin\theta)(0) = \cos\theta$$

2. **Cofactor Matrix $\text{Cof}(\mathbf{B})$**:
   $$\text{Cof}(\mathbf{B}) = \begin{bmatrix} \cos\theta & 0 & 0 \\ \sin\phi\sin\theta & \cos\phi\cos\theta & \sin\phi \\ \cos\phi\sin\theta & -\sin\phi\cos\theta & \cos\phi \end{bmatrix}$$

3. **Adjugate Matrix $\text{adj}(\mathbf{B}) = \text{Cof}(\mathbf{B})^T$**:
   $$\text{adj}(\mathbf{B}) = \begin{bmatrix} \cos\theta & \sin\phi\sin\theta & \cos\phi\sin\theta \\ 0 & \cos\phi\cos\theta & -\sin\phi\cos\theta \\ 0 & \sin\phi & \cos\phi \end{bmatrix}$$

4. **Dividing by $\det(\mathbf{B}) = \cos\theta$**:
   $$\mathbf{T}_\Theta(\boldsymbol{\eta}_2) = \begin{bmatrix} 1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\ 0 & \cos\phi & -\sin\phi \\ 0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta} \end{bmatrix}$$

#### Scalar Angular Kinematic Equations:
$$\dot{\phi} = p + q(\sin\phi\tan\theta) + r(\cos\phi\tan\theta)$$
$$\dot{\theta} = q(\cos\phi) - r(\sin\phi)$$
$$\dot{\psi} = q\left(\frac{\sin\phi}{\cos\theta}\right) + r\left(\frac{\cos\phi}{\cos\theta}\right)$$

---

### 3.3 Singularities, Gimbal Lock, and Unit Quaternion Formulation

When the vehicle's pitch angle reaches $\theta = \pm 90^\circ$, $\cos\theta = 0$ and $\tan\theta \to \pm\infty$, causing numerical singularity (gimbal lock). To avoid this in non-linear trajectory simulation, unit quaternions $\mathbf{q} = [\eta, \epsilon_1, \epsilon_2, \epsilon_3]^T \in \mathcal{S}^3$ satisfying $\eta^2 + \epsilon_1^2 + \epsilon_2^2 + \epsilon_3^2 = 1$ are utilized:

$$\begin{bmatrix} \dot{\eta} \\ \dot{\boldsymbol{\epsilon}} \end{bmatrix} = \frac{1}{2} \begin{bmatrix} -\boldsymbol{\epsilon}^T \\ \eta\mathbf{I}_{3\times 3} + \mathbf{S}(\boldsymbol{\epsilon}) \end{bmatrix} \boldsymbol{\nu}_2 = \frac{1}{2}\mathbf{E}(\mathbf{q})\boldsymbol{\nu}_2$$

The corresponding rotation matrix expressed in quaternions is:

$$\mathbf{R}(\mathbf{q}) = (\eta^2 - \boldsymbol{\epsilon}^T\boldsymbol{\epsilon})\mathbf{I}_{3\times 3} + 2\boldsymbol{\epsilon}\boldsymbol{\epsilon}^T + 2\eta\mathbf{S}(\boldsymbol{\epsilon})$$

---

### 3.4 Full $6 \times 6$ Kinematic Jacobian Matrix $\mathbf{J}(\boldsymbol{\eta}_2)$

Assembling $\mathbf{R}_b^n(\boldsymbol{\eta}_2)$ and $\mathbf{T}_\Theta(\boldsymbol{\eta}_2)$ into the $6 \times 6$ block diagonal matrix $\mathbf{J}(\boldsymbol{\eta}_2) = \text{diag}[\mathbf{R}_b^n, \mathbf{T}_\Theta]$:

$$\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{z} \\ \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 
\cos\psi\cos\theta & -\sin\psi\cos\phi + \cos\psi\sin\theta\sin\phi & \sin\psi\sin\phi + \cos\psi\sin\theta\cos\phi & 0 & 0 & 0 \\ 
\sin\psi\cos\theta & \cos\psi\cos\phi + \sin\psi\sin\theta\sin\phi & -\cos\psi\sin\phi + \sin\psi\sin\theta\cos\phi & 0 & 0 & 0 \\ 
-\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi & 0 & 0 & 0 \\ 
0 & 0 & 0 & 1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\ 
0 & 0 & 0 & 0 & \cos\phi & -\sin\phi \\ 
0 & 0 & 0 & 0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta} 
\end{bmatrix} \begin{bmatrix} u \\ v \\ w \\ p \\ q \\ r \end{bmatrix}$$

---

## 4. Exhaustive First-Principles Derivation of 6-DOF Dynamics (BlueROV2 Heavy Plant)

Fossen's full 6-DOF marine craft kinetics equation is formulated as:

$$\mathbf{M}\dot{\boldsymbol{\nu}} + \mathbf{C}_{RB}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{C}_A(\boldsymbol{\nu}_r)\boldsymbol{\nu}_r + \mathbf{D}(\boldsymbol{\nu}_r)\boldsymbol{\nu}_r + \mathbf{g}(\boldsymbol{\eta}) = \boldsymbol{\tau} + \boldsymbol{\tau}_{\text{ext}}$$

---

### 4.1 Total Mass Tensor $\mathbf{M} = \mathbf{M}_{RB} + \mathbf{M}_A$

#### 4.1.1 Rigid-Body Mass Matrix $\mathbf{M}_{RB}$
Formulated about Center of Origin $O_b$ with Center of Gravity offset $\mathbf{r}_g = [x_g, y_g, z_g]^T$:

$$\mathbf{M}_{RB} = \begin{bmatrix} m\mathbf{I}_{3\times 3} & -m\mathbf{S}(\mathbf{r}_g) \\ m\mathbf{S}(\mathbf{r}_g) & \mathbf{I}_g - m\mathbf{S}^2(\mathbf{r}_g) \end{bmatrix} \in \mathbb{R}^{6\times 6}$$

where $m = 13.5\,\text{kg}$ is dry mass, and $\mathbf{I}_g$ is the rigid-body inertia tensor about CG:

$$\mathbf{I}_g = \begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz} \\ -I_{xy} & I_{yy} & -I_{yz} \\ -I_{xz} & -I_{yz} & I_{zz} \end{bmatrix} = \begin{bmatrix} 0.16 & 0 & 0 \\ 0 & 0.21 & 0 \\ 0 & 0 & 0.245 \end{bmatrix} \text{kg}\cdot\text{m}^2$$

Evaluating parallel axis moment term $-m\mathbf{S}^2(\mathbf{r}_g)$:

$$-\mathbf{S}^2(\mathbf{r}_g) = \begin{bmatrix} y_g^2 + z_g^2 & -x_g y_g & -x_g z_g \\ -x_g y_g & x_g^2 + z_g^2 & -y_g z_g \\ -x_g z_g & -y_g z_g & x_g^2 + y_g^2 \end{bmatrix}$$

When $O_b$ coincides with CG ($\mathbf{r}_g = \mathbf{0}_{3\times 1}$) and leveraging structural symmetry ($I_{xy} = I_{xz} = I_{yz} = 0$), $\mathbf{M}_{RB}$ simplifies to:

$$\mathbf{M}_{RB} = \text{diag}[m, m, m, I_{xx}, I_{yy}, I_{zz}] = \text{diag}[13.5, 13.5, 13.5, 0.16, 0.21, 0.245]$$

#### 4.1.2 Hydrodynamic Added Mass Matrix $\mathbf{M}_A$
Added mass models virtual fluid inertia accelerated along with the hull. Derived from potential flow velocity potential $\phi$ satisfying Laplace's equation $\nabla^2\phi = 0$:

$$\mathbf{M}_A = -\begin{bmatrix} 
X_{\dot{u}} & X_{\dot{v}} & X_{\dot{w}} & X_{\dot{p}} & X_{\dot{q}} & X_{\dot{r}} \\ 
Y_{\dot{u}} & Y_{\dot{v}} & Y_{\dot{w}} & Y_{\dot{p}} & Y_{\dot{q}} & Y_{\dot{r}} \\ 
Z_{\dot{u}} & Z_{\dot{v}} & Z_{\dot{w}} & Z_{\dot{p}} & Z_{\dot{q}} & Z_{\dot{r}} \\ 
K_{\dot{u}} & K_{\dot{v}} & K_{\dot{w}} & K_{\dot{p}} & K_{\dot{q}} & K_{\dot{r}} \\ 
M_{\dot{u}} & M_{\dot{v}} & M_{\dot{w}} & M_{\dot{p}} & M_{\dot{q}} & M_{\dot{r}} \\ 
N_{\dot{u}} & N_{\dot{v}} & N_{\dot{w}} & N_{\dot{p}} & N_{\dot{q}} & N_{\dot{r}} 
\end{bmatrix}$$

For a symmetrical open-frame ROV at slow speeds, off-diagonal terms vanish, yielding:

$$\mathbf{M}_A = -\text{diag}[X_{\dot{u}}, Y_{\dot{v}}, Z_{\dot{w}}, K_{\dot{p}}, M_{\dot{q}}, N_{\dot{r}}] = \text{diag}[6.36, 7.12, 18.68, 0.015, 0.080, 0.245]\,\text{kg}$$

#### 4.1.3 Combined System Mass Tensor $\mathbf{M}$
$$\mathbf{M} = \text{diag}[m - X_{\dot{u}}, m - Y_{\dot{v}}, m - Z_{\dot{w}}, I_{xx} - K_{\dot{p}}, I_{yy} - M_{\dot{q}}, I_{zz} - N_{\dot{r}}]$$
$$\mathbf{M} = \text{diag}[19.86\,\text{kg}, 20.62\,\text{kg}, 32.18\,\text{kg}, 0.175\,\text{kg}\cdot\text{m}^2, 0.290\,\text{kg}\cdot\text{m}^2, 0.490\,\text{kg}\cdot\text{m}^2]$$

---

### 4.2 Coriolis and Centripetal Matrices

#### 4.2.1 Rigid-Body Coriolis Matrix $\mathbf{C}_{RB}(\boldsymbol{\nu})$
When $\mathbf{r}_g = \mathbf{0}$, $\mathbf{C}_{RB}(\boldsymbol{\nu})$ expands to:

$$\mathbf{C}_{RB}(\boldsymbol{\nu}) = \begin{bmatrix} 
0 & 0 & 0 & 0 & mw & -mv \\ 
0 & 0 & 0 & -mw & 0 & mu \\ 
0 & 0 & 0 & mv & -mu & 0 \\ 
0 & mw & -mv & 0 & I_{zz}r & -I_{yy}q \\ 
-mw & 0 & mu & -I_{zz}r & 0 & I_{xx}p \\ 
mv & -mu & 0 & I_{yy}q & -I_{xx}p & 0 
\end{bmatrix}$$

#### 4.2.2 Added Mass Coriolis Matrix $\mathbf{C}_A(\boldsymbol{\nu}_r)$
$$\mathbf{C}_A(\boldsymbol{\nu}_r) = \begin{bmatrix} 
0 & 0 & 0 & 0 & -Z_{\dot{w}}w_r & Y_{\dot{v}}v_r \\ 
0 & 0 & 0 & Z_{\dot{w}}w_r & 0 & -X_{\dot{u}}u_r \\ 
0 & 0 & 0 & -Y_{\dot{v}}v_r & X_{\dot{u}}u_r & 0 \\ 
0 & -Z_{\dot{w}}w_r & Y_{\dot{v}}v_r & 0 & -N_{\dot{r}}r & M_{\dot{q}}q \\ 
Z_{\dot{w}}w_r & 0 & -X_{\dot{u}}u_r & N_{\dot{r}}r & 0 & -K_{\dot{p}}p \\ 
-Y_{\dot{v}}v_r & X_{\dot{u}}u_r & 0 & -M_{\dot{q}}q & K_{\dot{p}}p & 0 
\end{bmatrix}$$

---

### 4.3 Hydrodynamic Damping Tensor $\mathbf{D}(\boldsymbol{\nu}_r)$

Hydrodynamic drag combines linear skin friction $\mathbf{D}_L$ and quadratic drag $\mathbf{D}_{NL}(\boldsymbol{\nu}_r)$:

$$\mathbf{D}_L = -\text{diag}[X_u, Y_v, Z_w, K_p, M_q, N_r]$$
$$\mathbf{D}_{NL}(\boldsymbol{\nu}_r) = -\text{diag}[X_{u|u|}|u_r|, Y_{v|v|}|v_r|, Z_{w|w|}|w_r|, K_{p|p|}|p|, M_{q|q|}|q|, N_{r|r|}|r|]$$

Multiplying $\mathbf{D}(\boldsymbol{\nu}_r)$ by relative velocity $\boldsymbol{\nu}_r$:

$$\mathbf{D}(\boldsymbol{\nu}_r)\boldsymbol{\nu}_r = \begin{bmatrix} 
-(X_u + X_{u|u|}|u_r|)u_r \\ 
-(Y_v + Y_{v|v|}|v_r|)v_r \\ 
-(Z_w + Z_{w|w|}|w_r|)w_r \\ 
-(K_p + K_{p|p|}|p|)p \\ 
-(M_q + M_{q|q|}|q|)q \\ 
-(N_r + N_{r|r|}|r|)r 
\end{bmatrix}$$

Numerical parameters for BlueROV2:
- **Linear damping**: $X_u = -4.03\,\text{Ns/m}$, $Y_v = -6.22\,\text{Ns/m}$, $Z_w = -5.18\,\text{Ns/m}$, $N_r = -0.50\,\text{Ns}\cdot\text{m/rad}$.
- **Quadratic damping**: $X_{u|u|} = -18.18\,\text{Ns}^2/\text{m}^2$, $Y_{v|v|} = -21.66\,\text{Ns}^2/\text{m}^2$, $Z_{w|w|} = -36.99\,\text{Ns}^2/\text{m}^2$, $N_{r|r|} = -1.55\,\text{Ns}^2/\text{rad}^2$.

---

### 4.4 Full 6-DOF Hydrostatic Restoring Vector $\mathbf{g}(\boldsymbol{\eta})$

Hydrostatic forces combine gravitational weight $W = mg$ acting downward at CG and buoyant force $B = \rho g \nabla$ acting upward at CB:

$$\mathbf{f}_g^b = \mathbf{R}_n^b \begin{bmatrix} 0 \\ 0 \\ W \end{bmatrix} = \begin{bmatrix} -W\sin\theta \\ W\cos\theta\sin\phi \\ W\cos\theta\cos\phi \end{bmatrix}, \qquad \mathbf{f}_b^b = \mathbf{R}_n^b \begin{bmatrix} 0 \\ 0 \\ -B \end{bmatrix} = \begin{bmatrix} B\sin\theta \\ -B\cos\theta\sin\phi \\ -B\cos\theta\cos\phi \end{bmatrix}$$

Evaluating moments $\boldsymbol{\tau}_g = \mathbf{r}_g \times \mathbf{f}_g^b$ and $\boldsymbol{\tau}_b = \mathbf{r}_b \times \mathbf{f}_b^b$:

$$\mathbf{g}(\boldsymbol{\eta}) = \begin{bmatrix} 
(W - B)\sin\theta \\ 
-(W - B)\cos\theta\sin\phi \\ 
-(W - B)\cos\theta\cos\phi \\ 
-(y_g W - y_b B)\cos\theta\cos\phi + (z_g W - z_b B)\cos\theta\sin\phi \\ 
(z_g W - z_b B)\sin\theta + (x_g W - x_b B)\cos\theta\cos\phi \\ 
-(x_g W - x_b B)\cos\theta\sin\phi - (y_g W - y_b B)\sin\theta 
\end{bmatrix}$$

When CO aligns with CB ($x_b = y_b = z_b = 0$) and CG is at $[0, 0, z_g]^T$ ($z_g > 0$):

$$\mathbf{g}(\boldsymbol{\eta}) = \begin{bmatrix} 
(W - B)\sin\theta \\ 
-(W - B)\cos\theta\sin\phi \\ 
-(W - B)\cos\theta\cos\phi \\ 
z_g W\cos\theta\sin\phi \\ 
z_g W\sin\theta \\ 
0 
\end{bmatrix}$$

---

### 4.5 Thruster Control Allocation for BlueROV2 Heavy (8 Thrusters)

For the 8-thruster Heavy configuration, the $6 \times 8$ Thruster Allocation Matrix $\mathbf{T}_{6\times 8}$ maps thruster forces $\mathbf{f} \in \mathbb{R}^8$ to 6-DOF body forces/moments $\boldsymbol{\tau} \in \mathbb{R}^6$:

$$\boldsymbol{\tau} = \mathbf{T}_{6\times 8} \mathbf{f}_{\text{thruster}}$$

Moore-Penrose pseudo-inverse allocation:

$$\mathbf{f}_{\text{thruster}} = \mathbf{T}_{6\times 8}^+ \boldsymbol{\tau}_{\text{demand}} = \mathbf{T}_{6\times 8}^T (\mathbf{T}_{6\times 8} \mathbf{T}_{6\times 8}^T)^{-1} \boldsymbol{\tau}_{\text{demand}}$$

derived from Lagrangian energy minimization $\min_{\mathbf{f}} \frac{1}{2}\mathbf{f}^T \mathbf{f}$ subject to $\boldsymbol{\tau} = \mathbf{T}_{6\times 8}\mathbf{f}$.

---

## 5. Unabridged Variable-by-Variable Reduction from 6-DOF to Decoupled 4-DOF (BlueROV2 Standard)

To demonstrate the step-by-step reduction of the 6-DOF system to a decoupled 4-DOF state-space model for the BlueROV2 Standard, every single variable, sub-variable, matrix row, matrix column, and scalar equation is reduced individually.

### 5.1 Physical Reduction Mechanism
Vertical metacentric separation ($GM_T = z_g - z_b \approx 0.020\,\text{m} > 0$) generates physical restoring torques $K_{\text{restoring}} = -z_g W\sin\phi$ and $M_{\text{restoring}} = -z_g W\sin\theta$ that act as torsional stiffness springs $k_\phi = z_g W$ and $k_\theta = z_g W$. Under small-angle operation, these springs constrain roll and pitch:

$$\phi \to 0, \quad \theta \to 0, \quad p \to 0, \quad q \to 0, \quad \dot{p} \to 0, \quad \dot{q} \to 0$$

---

### 5.2 Variable-by-Variable Kinematic Reduction

#### 1. State Vector Reduction
$$\boldsymbol{\eta}_{\text{6-DOF}} = \begin{bmatrix} x \\ y \\ z \\ \phi \\ \theta \\ \psi \end{bmatrix} \xrightarrow{\phi=0,\theta=0} \boldsymbol{\eta}_{\text{4-DOF}} = \begin{bmatrix} x \\ y \\ z \\ \psi \end{bmatrix} \in \mathbb{R}^4$$

$$\boldsymbol{\nu}_{\text{6-DOF}} = \begin{bmatrix} u \\ v \\ w \\ p \\ q \\ r \end{bmatrix} \xrightarrow{p=0,q=0} \boldsymbol{\nu}_{\text{4-DOF}} = \begin{bmatrix} u \\ v \\ w \\ r \end{bmatrix} \in \mathbb{R}^4$$

#### 2. Linear Rotation Matrix Reduction $\mathbf{R}_b^n \to \mathbf{R}_4$
Evaluating every single cell $R_{ij}$ of $\mathbf{R}_b^n(\boldsymbol{\eta}_2)$ at $\phi = 0$ and $\theta = 0$ ($\cos 0 = 1, \sin 0 = 0$):
- $R_{11} = \cos\psi\cos(0) = \cos\psi$
- $R_{12} = -\sin\psi\cos(0) + \cos\psi\sin(0)\sin(0) = -\sin\psi$
- $R_{13} = \sin\psi\sin(0) + \cos\psi\sin(0)\cos(0) = 0$
- $R_{21} = \sin\psi\cos(0) = \sin\psi$
- $R_{22} = \cos\psi\cos(0) + \sin\psi\sin(0)\sin(0) = \cos\psi$
- $R_{23} = -\cos\psi\sin(0) + \sin\psi\sin(0)\cos(0) = 0$
- $R_{31} = -\sin(0) = 0$
- $R_{32} = \cos(0)\sin(0) = 0$
- $R_{33} = \cos(0)\cos(0) = 1$

$$\mathbf{R}_b^n \Big|_{\phi=0,\theta=0} = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

#### 3. Angular Rate Transformation Matrix Reduction $\mathbf{T}_\Theta \to \mathbf{I}_{3\times 3}$
Evaluating every single cell $T_{ij}$ of $\mathbf{T}_\Theta(\boldsymbol{\eta}_2)$ at $\phi = 0$ and $\theta = 0$:
- $T_{11} = 1, \quad T_{12} = \sin(0)\tan(0) = 0, \quad T_{13} = \cos(0)\tan(0) = 0$
- $T_{21} = 0, \quad T_{22} = \cos(0) = 1, \quad T_{23} = -\sin(0) = 0$
- $T_{31} = 0, \quad T_{32} = \frac{\sin(0)}{\cos(0)} = 0, \quad T_{33} = \frac{\cos(0)}{\cos(0)} = 1$

$$\mathbf{T}_\Theta \Big|_{\phi=0,\theta=0} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} \implies \begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} p \\ q \\ r \end{bmatrix}$$

Because $p = 0$ and $q = 0$, $\dot{\phi} = 0$ and $\dot{\theta} = 0$, leaving $\dot{\psi} = r$.

#### 4. Kinematic Jacobian Reduction $\mathbf{J} \to \mathbf{J}_4$
Partitioning rows and columns 4 and 5 corresponding to $\phi, \theta, p, q$ yields:

$$\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{z} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 
\cos\psi & -\sin\psi & 0 & 0 \\ 
\sin\psi & \cos\psi & 0 & 0 \\ 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1 
\end{bmatrix} \begin{bmatrix} u \\ v \\ w \\ r \end{bmatrix} \iff \dot{\boldsymbol{\eta}}_4 = \mathbf{J}_4(\psi)\boldsymbol{\nu}_4$$

---

### 5.3 Variable-by-Variable Dynamic Reduction (Kinetics)

#### 1. Mass Tensor Reduction $\mathbf{M} \to \mathbf{M}_4$
Deleting rows 4 and 5 (roll and pitch moment equations) and columns 4 and 5 (roll angular acceleration $\dot{p}$ and pitch angular acceleration $\dot{q}$):

$$\mathbf{M}_{\text{4-DOF}} = \begin{bmatrix} 
m - X_{\dot{u}} & 0 & 0 & 0 \\ 
0 & m - Y_{\dot{v}} & 0 & 0 \\ 
0 & 0 & m - Z_{\dot{w}} & 0 \\ 
0 & 0 & 0 & I_{zz} - N_{\dot{r}} 
\end{bmatrix} = \begin{bmatrix} 
19.86 & 0 & 0 & 0 \\ 
0 & 20.62 & 0 & 0 \\ 
0 & 0 & 32.18 & 0 \\ 
0 & 0 & 0 & 0.490 
\end{bmatrix}$$

#### 2. Coriolis Tensor Reduction and First-Principles Munk Moment Proof
Multiplying the full 6-DOF Coriolis matrix $(\mathbf{C}_{RB} + \mathbf{C}_A)$ by relative velocity $\boldsymbol{\nu}_r = [u_r, v_r, w_r, p, q, r]^T$ under $p = 0, q = 0$:

1. **Surge Row 1**:
   $$\tau_{X,\text{Coriolis}} = 0\cdot u_r + 0\cdot v_r + 0\cdot w_r + 0\cdot p + (m w - Z_{\dot{w}}w_r)q^0 - (m v - Y_{\dot{v}}v_r)r = -(m - Y_{\dot{v}})v_r r$$

2. **Sway Row 2**:
   $$\tau_{Y,\text{Coriolis}} = 0\cdot u_r + 0\cdot v_r + 0\cdot w_r - (m w - Z_{\dot{w}}w_r)p^0 + 0\cdot q + (m u - X_{\dot{u}}u_r)r = (m - X_{\dot{u}})u_r r$$

3. **Heave Row 3**:
   $$\tau_{Z,\text{Coriolis}} = 0\cdot u_r + 0\cdot v_r + 0\cdot w_r + (m v - Y_{\dot{v}}v_r)p^0 - (m u - X_{\dot{u}}u_r)q^0 + 0\cdot r = 0$$

4. **Roll Row 4**: Deleted ($p = 0, \dot{p} = 0$).
5. **Pitch Row 5**: Deleted ($q = 0, \dot{q} = 0$).

6. **Yaw Row 6**:
   $$\tau_{N,\text{Coriolis}} = -(m v - Y_{\dot{v}}v_r)u_r + (m u - X_{\dot{u}}u_r)v_r + 0\cdot w_r - (I_{yy}q - M_{\dot{q}}q)p^0 + (I_{xx}p - K_{\dot{p}}p)q^0 + 0\cdot r$$
   $$= -m v_r u_r + Y_{\dot{v}}v_r u_r + m u_r v_r - X_{\dot{u}}u_r v_r$$
   $$= (X_{\dot{u}} - Y_{\dot{v}})u_r v_r \quad \text{\textbf{(The Hydrodynamic Munk Moment)}}$$

Substituting numerical added mass derivatives:
$$X_{\dot{u}} - Y_{\dot{v}} = -6.36 - (-7.12) = +0.76 > 0$$

> [!CAUTION]
> **The Destabilizing Munk Moment**: Because transverse added mass $|Y_{\dot{v}}| = 7.12\,\text{kg}$ exceeds surge added mass $|X_{\dot{u}}| = 6.36\,\text{kg}$, the Munk gain is positive ($+0.76\,\text{kg}$). When cruising forward ($u_r > 0$) with slight cross-flow ($v_r > 0$), this hydrodynamic torque attempts to rotate the vehicle broadside to the flow, requiring active yaw feedback control ($N$) to maintain heading.

#### 3. Hydrodynamic Damping Reduction
Evaluating $\mathbf{D}(\boldsymbol{\nu}_r)\boldsymbol{\nu}_r$ under $p = 0, q = 0$:
- **Surge Damping**: $-(X_u + X_{u|u|}|u_r|)u_r$
- **Sway Damping**: $-(Y_v + Y_{v|v|}|v_r|)v_r$
- **Heave Damping**: $-(Z_w + Z_{w|w|}|w_r|)w_r$
- **Roll Damping**: $-(K_p + K_{p|p|}|p^0|)p^0 = 0$ (Eliminated)
- **Pitch Damping**: $-(M_q + M_{q|q|}|q^0|)q^0 = 0$ (Eliminated)
- **Yaw Damping**: $-(N_r + N_{r|r|}|r|)r$

#### 4. Hydrostatic Restoring Vector Reduction $\mathbf{g} \to \mathbf{g}_4$
Evaluating $\mathbf{g}(\boldsymbol{\eta})$ at $\phi = 0$ and $\theta = 0$:
- Surge Row 1: $(W - B)\sin(0) = 0$
- Sway Row 2: $-(W - B)\cos(0)\sin(0) = 0$
- Heave Row 3: $-(W - B)\cos(0)\cos(0) = -(W - B) = B - W = \rho g \nabla - mg$
- Roll Row 4: $z_g W\cos(0)\sin(0) = 0$
- Pitch Row 5: $z_g W\sin(0) = 0$
- Yaw Row 6: $0$

Deleting rows 4 and 5 yields:

$$\mathbf{g}_4 = \begin{bmatrix} 0 \\ 0 \\ \rho g \nabla - mg \\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ -2.306 \\ 0 \end{bmatrix} \text{N}$$

#### 5. Control Force Vector and Allocation Reduction
Setting unactuated moments $\tau_K = 0$ and $\tau_M = 0$, the $6 \times 8$ thruster matrix reduces to the $4 \times 6$ matrix $\mathbf{T}_{4\times 6}$:

$$\mathbf{f}_{\text{thruster, Standard}} = \mathbf{T}_{4\times 6}^+ \boldsymbol{\tau}_{\text{demand}} = \mathbf{T}_{4\times 6}^T (\mathbf{T}_{4\times 6}\mathbf{T}_{4\times 6}^T)^{-1} \boldsymbol{\tau}_{\text{demand}} \in \mathbb{R}^6$$

---

### 5.4 Unified Decoupled 4-DOF Non-linear Differential Equations

Combining all variable-by-variable reductions yields the complete 4-DOF plant governing the BlueROV2 Standard:

$$\text{\textbf{Surge:}} \quad (m - X_{\dot{u}})\dot{u} - (m - Y_{\dot{v}})v_r r - (X_u + X_{u|u|}|u_r|)u_r = \tau_X$$

$$\text{\textbf{Sway:}} \quad (m - Y_{\dot{v}})\dot{v} + (m - X_{\dot{u}})u_r r - (Y_v + Y_{v|v|}|v_r|)v_r = \tau_Y$$

$$\text{\textbf{Heave:}} \quad (m - Z_{\dot{w}})\dot{w} - (Z_w + Z_{w|w|}|w_r|)w_r + (\rho g \nabla - mg) = \tau_Z$$

$$\text{\textbf{Yaw:}} \quad (I_{zz} - N_{\dot{r}})\dot{r} + (X_{\dot{u}} - Y_{\dot{v}})u_r v_r - (N_r + N_{r|r|}|r|)r = \tau_N$$

---

## 6. ArduSub Software Architecture, SITL Frame Mapping, and Control Architecture

### 6.1 ArduSub Autopilot Architecture & Telemetry Bridge
The BlueROV2 control stack connects lower-level motor mixing with high-level mission planning via a bi-directional telemetry architecture:
- **Flight Controller**: Pixhawk 2.4.8 running ArduSub firmware.
- **Companion Computer**: Raspberry Pi 4B running BlueOS / MAVProxy, bridging telemetry over Ethernet (`192.168.2.2`).
- **Control Modes**: `MANUAL`, `ALT_HOLD` (barometer depth holding), `POSHOLD` (UGPS/acoustic positioning), and `GUIDED` modes executing PID feedback control loops over IMU, barometer, and visual tracking signals.

### 6.2 ArduSub Frame Parameters and Mixer Matrices
ArduSub configures thruster control allocation using the `FRAME_CLASS` parameter:
- **Standard BlueROV2 (`-f vectored`)**: Configures 6 motor channels (4 horizontal vectored at $45^\circ$, 2 vertical). The firmware executes feedback control in 4 active DOFs (Surge, Sway, Heave, Yaw). Roll is kept level automatically by equalizing vertical thruster bias, while pitch is unactuated and relies on passive metacentric stability ($z_g > z_b$).
- **BlueROV2 Heavy (`-f vectored_6dof`)**: Configures 8 motor channels (4 horizontal vectored, 4 vertical corner thrusters). The firmware executes active feedback loops across all 6 DOFs, enabling pitch angle target holding (`PITCH_HOLD`) and active attitude stabilization.

### 6.3 Gazebo SITL Physics Simulation Integration
In Software-In-The-Loop (SITL) simulations (Gazebo Harmonic with ROS 2 Jazzy), vehicle SDF models (`model.sdf`) define:
- Hydrodynamic added mass tensors ($\mathbf{M}_A$) and non-linear damping matrices ($\mathbf{D}$).
- Buoyancy plugin calculations calculating net buoyant force $B = \rho g \nabla$ and moment arm $\mathbf{r}_b - \mathbf{r}_g$.
- ArduSub SITL plugin mapping motor PWM values ($1100\,\mu\text{s} \dots 1900\,\mu\text{s}$) to thrust forces $F = K_T \rho D_p^4 n |n|$.

---

## 7. Comparative Architectural Synthesis: BlueROV2 Heavy (6-DOF) vs. BlueROV2 Standard (4-DOF)

| Technical Metric | BlueROV2 Heavy (6-DOF Active Plant) | BlueROV2 Standard (Decoupled 4-DOF Reduction) |
|---|---|---|
| **Thruster Count** | 8 T200 Brushless Thrusters | 6 T200 Brushless Thrusters |
| **Horizontal Layout** | 4 Vectored at $45^\circ$ angles in $x_b-y_b$ plane | 4 Vectored at $45^\circ$ angles in $x_b-y_b$ plane |
| **Vertical Layout** | 4 Vertical corner thrusters (port/stbd fore/aft) | 2 Side-by-side vertical thrusters along $y_b$ axis |
| **ArduSub Frame Setting** | `-f vectored_6dof` | `-f vectored` |
| **Active Controllable DOFs** | Full 6-DOF (Surge, Sway, Heave, Roll, Pitch, Yaw) | 4-DOF (Surge, Sway, Heave, Yaw) |
| **Pitch Axis ($\theta$) Authority** | Active differential thrust torque authority | Unactuated; passive metacentric righting spring |
| **Roll Axis ($\phi$) Authority** | Active differential thrust torque authority | Passive righting / equalized vertical thruster bias |
| **Allocation Matrix Size** | $6 \times 8$ matrix ($\text{rank} = 6$) | $4 \times 6$ matrix ($\text{rank} = 4$) |
| **State Vector Dimension** | $\boldsymbol{\eta} \in \mathbb{R}^6, \boldsymbol{\nu} \in \mathbb{R}^6$ | $\boldsymbol{\eta}_4 \in \mathbb{R}^4, \boldsymbol{\nu}_4 \in \mathbb{R}^4$ |
| **Cross-Coupling Dynamics** | Full $6 \times 6$ non-linear Coriolis coupling | Unstable Munk Moment $(X_{\dot{u}} - Y_{\dot{v}})u_r v_r$ in Yaw |
| **Inspection Capability** | Pitch holding (pitch nose up/down at fixed depth) | Horizontal orientation; fixed pitch angle |
| **Manipulator Arm Operation** | High stability under manipulator reaction torques | Pitch disturbance under heavy arm load |

---

## 8. Hierarchical Variable-Within-Variable Parameter Taxonomy Dictionary

| Variable / Sub-Variable | Physical Definition and Derivation Source | Nominal Value / Unit |
|---|---|---|
| $m$ | Dry physical mass of vehicle ($m = \int_V \rho_{\text{body}} dV$) | $13.5\,\text{kg}$ |
| $\rho$ | Fluid mass density of seawater | $1025\,\text{kg/m}^3$ |
| $\mu$ | Fluid dynamic viscosity | $1.002 \times 10^{-3}\,\text{Pa}\cdot\text{s}$ |
| $\nabla$ | Displaced volume of water ($\nabla = \int_\nabla dV$) | $0.0134\,\text{m}^3$ |
| $g$ | Acceleration due to gravity constant | $9.81\,\text{m/s}^2$ |
| $W$ | Gravitational weight force ($W = mg$) | $132.44\,\text{N}$ |
| $B$ | Hydrostatic buoyant force ($B = \rho g \nabla$) | $134.74\,\text{N}$ |
| $B - W$ | Net vertical positive buoyancy force | $+2.31\,\text{N}$ |
| $\mathbf{r}_g = [x_g, y_g, z_g]^T$ | CG position offset relative to origin $O_b$ | $[0, 0, 0.02]^T\,\text{m}$ |
| $\mathbf{r}_b = [x_b, y_b, z_b]^T$ | CB position offset relative to origin $O_b$ | $[0, 0, 0.00]^T\,\text{m}$ |
| $z_g - z_b$ | Metacentric height separating CB and CG | $0.020\,\text{m}$ |
| $I_{xx}, I_{yy}, I_{zz}$ | Rigid moments of inertia ($I_{zz} = \int (x^2 + y^2)dm$) | $0.16, 0.21, 0.245\,\text{kg}\cdot\text{m}^2$ |
| $X_{\dot{u}}, Y_{\dot{v}}, Z_{\dot{w}}$ | Hydrodynamic added mass ($M_{A,ij} = -\rho \iint \phi_i \frac{\partial\phi_j}{\partial n} dS$) | $-6.36, -7.12, -18.68\,\text{kg}$ |
| $K_{\dot{p}}, M_{\dot{q}}, N_{\dot{r}}$ | Hydrodynamic added rotational inertia | $-0.015, -0.080, -0.245\,\text{kg}\cdot\text{m}^2$ |
| $m_u, m_v, m_w$ | Virtual total masses ($m_u = m - X_{\dot{u}}$) | $19.86, 20.62, 32.18\,\text{kg}$ |
| $I_r$ | Virtual total yaw inertia ($I_r = I_{zz} - N_{\dot{r}}$) | $0.490\,\text{kg}\cdot\text{m}^2$ |
| $X_{\dot{u}} - Y_{\dot{v}}$ | Munk Moment cross-coupling yaw torque gain | $+0.76\,\text{kg}$ |
| $X_u, Y_v, Z_w$ | Linear skin friction damping derivatives | $-4.03, -6.22, -5.18\,\text{Ns/m}$ |
| $N_r$ | Linear yaw angular friction derivative | $-0.50\,\text{Ns}\cdot\text{m/rad}$ |
| $X_{u|u|}, Y_{v|v|}, Z_{w|w|}$ | Quadratic drag coefficients ($\frac{1}{2}\rho C_d A_c$) | $-18.18, -21.66, -36.99\,\text{Ns}^2/\text{m}^2$ |
| $N_{r|r|}$ | Quadratic yaw moment drag coefficient | $-1.55\,\text{Ns}^2/\text{rad}^2$ |
| $D_p$ | T200 propeller diameter | $0.076\,\text{m}$ |
| $K_T(J_a)$ | Dimensionless thrust coefficient | $0.11$ |
| $w_T$ | Taylor wake fraction | $0.10$ |
| $\mathbf{T}_{6\times 8}^+$ | Heavy 8-thruster pseudo-inverse allocation matrix | $8 \times 6$ matrix |
| $\mathbf{T}_{4\times 6}^+$ | Standard 6-thruster pseudo-inverse allocation matrix | $6 \times 4$ matrix |

---

## 9. References

1. Fossen, T. I. (2021). *Handbook of Marine Craft Hydrodynamics and Motion Control* (2nd ed.). John Wiley & Sons.
2. SNAME (1950). *Nomenclature for Treating the Motion of Vessels Through Fluids*. The Society of Naval Architects and Marine Engineers, Technical and Research Bulletin No. 1-5.
3. Blue Robotics (2024). *BlueROV2 User Operating Guide and Technical Specifications*. Blue Robotics Inc., Torrance, CA.
4. von Benzon, M., et al. (2022). *Standardizing Fossen’s Equations of Motion for Remotely Operated Vehicles*. Aalborg University Technical Report.
5. Ng, W., & Krieg, M. (2024). *Hydrodynamic Identification and SITL Simulation Calibration of the BlueROV2*. University of Hawaii Technical Report.
6. Shafeeq, R. (2026). *AUV Development Repository: Autonomous Target Tracking and Visual Servoing Control*. GitHub Repository: [radshafeeq/AUV-Development](https://github.com/radshafeeq/AUV-Development).
