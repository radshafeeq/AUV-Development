# Comprehensive Theoretical Derivation and Engineering Analysis: Singular Kalman Filter, Extended Kalman Filter, and AUV Autonomous Closed-Loop Architecture

> **Author**: Radhi Shafeeq  
> **Affiliation**: Hasanuddin University — Department of Mechatronics Engineering  
> **Undergraduate Thesis**: Design, Hydrodynamic Modeling, State Estimation, and Autonomous Visual Servoing for a 5-DOF Autonomous Underwater Vehicle (AUV)  
> **Primary Academic Sources**: 
> 1. Fossen, T. I. (2021). *Handbook of Marine Craft Hydrodynamics and Motion Control* (2nd ed.). John Wiley & Sons.
> 2. Kim, Y. V. (Ed.). (2023). *Kalman Filter - Engineering Applications*. IntechOpen. DOI: 10.5772/intechopen.100722.
> 3. Khalid, A., Sarwat, A., & Riggs, H. (Eds.). (2024). *Applications and Optimizations of Kalman Filter and Their Variants*. IntechOpen. DOI: 10.5772/intechopen.109154.
> 4. Särkkä, S., & Svensson, L. (2023). *Bayesian Filtering and Smoothing* (2nd ed.). Cambridge University Press. DOI: 10.1017/9781108910002.
>
> **LaTeX Formatting Notice**: All mathematical equations and variables in this monograph are formatted with `$$...$$` delimiters for native, zero-error conversion into rendered publication graphics using Google Docs and the **Auto-LaTeX Equations** add-on.

---

## Master Table of Contents

- [Part I: General Theoretical Foundations & Complete Derivations](#part-i-general-theoretical-foundations--complete-derivations)
  - [1. Introduction to Optimal State Estimation & Stochastic Calculus](#1-introduction-to-optimal-state-estimation--stochastic-calculus)
    - [1.1 Probability Spaces, Random Vectors, and Second-Order Statistics](#11-probability-spaces-random-vectors-and-second-order-statistics)
    - [1.2 Multivariate Gaussian Distributions and Linear Invariance](#12-multivariate-gaussian-distributions-and-linear-invariance)
    - [1.3 The Minimum Mean-Square Error (MMSE) Criterion](#13-the-minimum-mean-square-error-mmse-criterion)
    - [1.4 The Orthogonality Principle in Hilbert Estimation Space](#14-the-orthogonality-principle-in-hilbert-estimation-space)
  - [2. The Singular (Linear Discrete) Kalman Filter — Complete First-Principles Derivation](#2-the-singular-linear-discrete-kalman-filter--complete-first-principles-derivation)
    - [2.1 Discrete-Time Linear State-Space Formulation](#21-discrete-time-linear-state-space-formulation)
    - [2.2 Step 1 Derivation: Prior State Prediction](#22-step-1-derivation-prior-state-prediction)
    - [2.3 Step 2 Derivation: Prior Error Covariance Matrix](#23-step-2-derivation-prior-error-covariance-matrix)
    - [2.4 Step 3 Derivation: Innovation Residual and Innovation Covariance](#24-step-3-derivation-innovation-residual-and-innovation-covariance)
    - [2.5 Step 4 Derivation: Posterior Update and the Joseph Form Covariance](#25-step-4-derivation-posterior-update-and-the-joseph-form-covariance)
    - [2.6 Step 5 Derivation: Calculus of Variations & Optimal Kalman Gain](#26-step-5-derivation-calculus-of-variations--optimal-kalman-gain)
    - [2.7 Algebraic Simplification of Posterior Covariance](#27-algebraic-simplification-of-posterior-covariance)
  - [3. The Extended Kalman Filter (EKF) — Complete First-Principles Derivation](#3-the-extended-kalman-filter-ekf--complete-first-principles-derivation)
    - [3.1 The Curse of Non-Linearity & Gaussian Breakdown](#31-the-curse-of-non-linearity--gaussian-breakdown)
    - [3.2 Multivariable Taylor Series Expansion](#32-multivariable-taylor-series-expansion)
    - [3.3 Analytical Derivation of State and Measurement Jacobians](#33-analytical-derivation-of-state-and-measurement-jacobians)
    - [3.4 The Discrete EKF Predict-Correct Recursive Equations](#34-the-discrete-ekf-predict-correct-recursive-equations)
    - [3.5 Continuous-Discrete Extended Kalman Filter (C-D EKF) and Differential Riccati Integration](#35-continuous-discrete-extended-kalman-filter-c-d-ekf-and-differential-riccati-integration)
- [Part II: Comprehensive Derivation of the AUV Kalman Filter Suite](#part-ii-comprehensive-derivation-of-the-auv-kalman-filter-suite)
  - [4. Topside Visual Target Kalman Filter (`AUVVisualKalmanFilter`)](#4-topside-visual-target-kalman-filter-auvvisualkalmanfilter)
    - [4.1 Pinhole Camera Geometry & Perspective Coordinate Projection](#41-pinhole-camera-geometry--perspective-coordinate-projection)
    - [4.2 8D State-Space Vector Formulation](#42-8d-state-space-vector-formulation)
    - [4.3 Continuous White Noise Acceleration (CWNA) Derivation](#43-continuous-white-noise-acceleration-cwna-derivation)
    - [4.4 Exact Discretization of State Transition Matrix A(Δt)](#44-exact-discretization-of-state-transition-matrix-at)
    - [4.5 Exact Discretization of Process Noise Covariance Q(Δt) via Matrix Exponential Integrals](#45-exact-discretization-of-process-noise-covariance-qt-via-matrix-exponential-integrals)
    - [4.6 Analytical Monocular Scale Rate & Surge Range-Rate Derivation](#46-analytical-monocular-scale-rate--surge-range-rate-derivation)
    - [4.7 Adaptive Confidence-Weighted Measurement Covariance R(conf)](#47-adaptive-confidence-weighted-measurement-covariance-rconf)
    - [4.8 Mahalanobis Distance Innovation Outlier Gating](#48-mahalanobis-distance-innovation-outlier-gating)
    - [4.9 Occlusion Bridging & Dead-Reckoning Mathematical Mechanics](#49-occlusion-bridging--dead-reckoning-mathematical-mechanics)
  - [5. Subsea Hydrodynamic Dynamics Extended Kalman Filter (`AUVDynamicsKalmanFilter`)](#5-subsea-hydrodynamic-dynamics-extended-kalman-filter-auvdynamicskalmanfilter)
    - [5.1 SNAME Coordinate Frames & Kinematic Reductions](#51-sname-coordinate-frames--kinematic-reductions)
    - [5.2 Fossen's 6-DOF Hydrodynamic Kinetics Equations](#52-fossens-6-dof-hydrodynamic-kinetics-equations)
    - [5.3 Variable-by-Variable 4-DOF Decoupled Reduction](#53-variable-by-variable-4-dof-decoupled-reduction)
    - [5.4 Generalized Inertia Matrix M: Rigid Body & Hydrodynamic Added Mass](#54-generalized-inertia-matrix-m-rigid-body--hydrodynamic-added-mass)
    - [5.5 Hydrodynamic Damping Matrix D(ν): Linear Skin Friction & Non-Linear Quadratic Form Drag](#55-hydrodynamic-damping-matrix-d-linear-skin-friction--non-linear-quadratic-form-drag)
    - [5.6 Ocean Current Disturbance Observer Formulation](#56-ocean-current-disturbance-observer-formulation)
    - [5.7 First-Principles Derivation of the Analytical 6x6 Continuous Jacobian Matrix F](#57-first-principles-derivation-of-the-analytical-6x6-continuous-jacobian-matrix-f)
    - [5.8 Cayley-Hamilton Discretization Φ = I + FΔt](#58-cayley-hamilton-discretization--i--ft)
    - [5.9 Multi-Sensor Innovation & Update on Raspberry Pi 4B](#59-multi-sensor-innovation--update-on-raspberry-pi-4b)
  - [6. Distributed Topside-Subsea Architecture & Real-Time Performance](#6-distributed-topside-subsea-architecture--real-time-performance)
    - [6.1 3-Tier Network Topology & Tether Protocol](#61-3-tier-network-topology--tether-protocol)
    - [6.2 Microsecond Execution Profiling & Algorithmic Complexity](#62-microsecond-execution-profiling--algorithmic-complexity)
    - [6.3 Hardware-in-the-Loop (HIL) Dry Benchtop Testing Methodology](#63-hardware-in-the-loop-hil-dry-benchtop-testing-methodology)
  - [7. Closed-Loop Visual Servoing & Hydrodynamic Munk Moment Suppression](#7-closed-loop-visual-servoing--hydrodynamic-munk-moment-suppression)
    - [7.1 Image-Based Visual Servoing (IBVS) Interaction Matrix](#71-image-based-visual-servoing-ibvs-interaction-matrix)
    - [7.2 Mathematical Proof: Munk Moment Destabilization Suppression via Filtered State Feedback](#72-mathematical-proof-munk-moment-destabilization-suppression-via-filtered-state-feedback)
  - [8. Numerical Walkthrough: 5-Cycle Matrix Arithmetic with Real Numbers](#8-numerical-walkthrough-5-cycle-matrix-arithmetic-with-real-numbers)
  - [9. Complete Parameter Taxonomy & Variable Hierarchy Dictionary](#9-complete-parameter-taxonomy--variable-hierarchy-dictionary)
  - [10. Comprehensive Master Bibliography](#10-comprehensive-master-bibliography)

---

# Part I: General Theoretical Foundations & Complete Derivations

## 1. Introduction to Optimal State Estimation & Stochastic Calculus

### 1.1 Probability Spaces, Random Vectors, and Second-Order Statistics

Consider an underlying complete probability space denoted by the triple $$(\Omega, \mathcal{F}, \mathbb{P})$$, where:
- $$\Omega$$ represents the sample space containing all elementary experimental outcomes $$\omega \in \Omega$$.
- $$\mathcal{F}$$ represents the $$\sigma$$-algebra of subsets of $$\Omega$$ defining the collection of measurable physical events.
- $$\mathbb{P}: \mathcal{F} \to [0, 1]$$ represents the probability measure assigning measure to events such that $$\mathbb{P}(\Omega) = 1$$.

Let a continuous-time or discrete-time physical state of a dynamical vehicle be modeled as an $$n$$-dimensional real random vector:
$$\mathbf{x}: \Omega \to \mathbb{R}^n$$

The **Mathematical Expectation** (first statistical moment) of $$\mathbf{x}$$ is given by the Lebesgue-Stieltjes integral over the state distribution:
$$\boldsymbol{\mu}_{\mathbf{x}} = \mathbb{E}[\mathbf{x}] = \int_{\mathbb{R}^n} \mathbf{x} p(\mathbf{x}) \, d\mathbf{x}$$

where $$p(\mathbf{x}): \mathbb{R}^n \to [0, \infty)$$ is the joint probability density function (PDF).

The **Error Covariance Matrix** (second central statistical moment), which measures the dispersion, uncertainty, and cross-variable correlations of the state vector about its expected mean, is defined as:
$$\mathbf{P}_{\mathbf{x}} = \text{Cov}(\mathbf{x}) = \mathbb{E}\left[ (\mathbf{x} - \mathbb{E}[\mathbf{x}]) (\mathbf{x} - \mathbb{E}[\mathbf{x}])^T \right] = \int_{\mathbb{R}^n} (\mathbf{x} - \boldsymbol{\mu}_{\mathbf{x}})(\mathbf{x} - \boldsymbol{\mu}_{\mathbf{x}})^T p(\mathbf{x}) \, d\mathbf{x}$$

$$\mathbf{P}_{\mathbf{x}} \in \mathbb{R}^{n \times n}$$ is inherently **symmetric** ($$\mathbf{P}_{\mathbf{x}} = \mathbf{P}_{\mathbf{x}}^T$$) and **positive semi-definite** ($$\mathbf{z}^T \mathbf{P}_{\mathbf{x}} \mathbf{z} \ge 0, \forall \mathbf{z} \in \mathbb{R}^n$$).

For two distinct random vectors $$\mathbf{x} \in \mathbb{R}^n$$ and $$\mathbf{y} \in \mathbb{R}^m$$, their cross-covariance matrix is given by:
$$\text{Cov}(\mathbf{x}, \mathbf{y}) = \boldsymbol{\Sigma}_{\mathbf{x}\mathbf{y}} = \mathbb{E}\left[ (\mathbf{x} - \boldsymbol{\mu}_{\mathbf{x}})(\mathbf{y} - \boldsymbol{\mu}_{\mathbf{y}})^T \right] \in \mathbb{R}^{n \times m}$$

If $$\boldsymbol{\Sigma}_{\mathbf{x}\mathbf{y}} = \mathbf{0}_{n \times m}$$, the vectors $$\mathbf{x}$$ and $$\mathbf{y}$$ are statistically **uncorrelated**.

---

### 1.2 Multivariate Gaussian Distributions and Linear Invariance

A random vector $$\mathbf{x} \in \mathbb{R}^n$$ follows a multivariate Gaussian (Normal) probability distribution, denoted $$\mathbf{x} \sim \mathcal{N}(\boldsymbol{\mu}, \mathbf{P})$$, if its joint probability density function is strictly parameterized by its mean vector $$\boldsymbol{\mu}$$ and symmetric positive-definite covariance matrix $$\mathbf{P} \succ 0$$:
$$p(\mathbf{x}) = \frac{1}{(2\pi)^{n/2} \det(\mathbf{P})^{1/2}} \exp\left( -\frac{1}{2} (\mathbf{x} - \boldsymbol{\mu})^T \mathbf{P}^{-1} (\mathbf{x} - \boldsymbol{\mu}) \right)$$

#### Theorem 1.1: Linear Invariance of Gaussian Random Vectors
Let $$\mathbf{x} \sim \mathcal{N}(\boldsymbol{\mu}_{\mathbf{x}}, \mathbf{P}_{\mathbf{x}})$$ be an $$n$$-dimensional Gaussian vector. Let $$\mathbf{y} \in \mathbb{R}^m$$ be defined by an affine linear transformation:
$$\mathbf{y} = \mathbf{A}\mathbf{x} + \mathbf{b}$$
where $$\mathbf{A} \in \mathbb{R}^{m \times n}$$ is a deterministic transformation matrix and $$\mathbf{b} \in \mathbb{R}^m$$ is a deterministic translation vector.

Then $$\mathbf{y}$$ is **strictly Gaussian distributed**:
$$\mathbf{y} \sim \mathcal{N}(\boldsymbol{\mu}_{\mathbf{y}}, \mathbf{P}_{\mathbf{y}})$$
where:
$$\boldsymbol{\mu}_{\mathbf{y}} = \mathbb{E}[\mathbf{A}\mathbf{x} + \mathbf{b}] = \mathbf{A}\mathbb{E}[\mathbf{x}] + \mathbf{b} = \mathbf{A}\boldsymbol{\mu}_{\mathbf{x}} + \mathbf{b}$$
$$\mathbf{P}_{\mathbf{y}} = \mathbb{E}\left[ (\mathbf{y} - \boldsymbol{\mu}_{\mathbf{y}})(\mathbf{y} - \boldsymbol{\mu}_{\mathbf{y}})^T \right] = \mathbb{E}\left[ (\mathbf{A}(\mathbf{x} - \boldsymbol{\mu}_{\mathbf{x}}))(\mathbf{A}(\mathbf{x} - \boldsymbol{\mu}_{\mathbf{x}}))^T \right] = \mathbf{A} \mathbb{E}\left[ (\mathbf{x} - \boldsymbol{\mu}_{\mathbf{x}})(\mathbf{x} - \boldsymbol{\mu}_{\mathbf{x}})^T \right] \mathbf{A}^T = \mathbf{A} \mathbf{P}_{\mathbf{x}} \mathbf{A}^T$$

This fundamental theorem guarantees that in linear systems subject to additive Gaussian noise, the true posterior probability distribution remains **closed under Gaussianity**, completely defined by propagating only the mean vector and covariance matrix.

---

### 1.3 The Minimum Mean-Square Error (MMSE) Criterion

Let $$\mathbf{x} \in \mathbb{R}^n$$ be an unobservable physical system state, and let $$\mathbf{Z}^k = \{\mathbf{z}_1, \mathbf{z}_2, \dots, \mathbf{z}_k\}$$ denote the complete historical filtration of noisy sensor measurements collected up to time $$k$$. 

We seek an optimal state estimator $$\hat{\mathbf{x}}(\mathbf{Z}^k)$$ that minimizes the scalar expected quadratic error penalty:
$$J = \mathbb{E}\left[ \|\mathbf{x} - \hat{\mathbf{x}}\|^2 \mid \mathbf{Z}^k \right] = \mathbb{E}\left[ (\mathbf{x} - \hat{\mathbf{x}})^T (\mathbf{x} - \hat{\mathbf{x}}) \mid \mathbf{Z}^k \right] = \text{Tr}\left( \mathbb{E}\left[ (\mathbf{x} - \hat{\mathbf{x}})(\mathbf{x} - \hat{\mathbf{x}})^T \mid \mathbf{Z}^k \right] \right)$$

#### Proof that the Conditional Mean is the Optimal MMSE Estimator:
Add and subtract the conditional mean $$\mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k]$$ inside the error quadratic form:
$$\mathbf{x} - \hat{\mathbf{x}} = (\mathbf{x} - \mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k]) + (\mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k] - \hat{\mathbf{x}})$$

Expanding the inner product:
$$\|\mathbf{x} - \hat{\mathbf{x}}\|^2 = \|\mathbf{x} - \mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k]\|^2 + \|\mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k] - \hat{\mathbf{x}}\|^2 + 2 (\mathbf{x} - \mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k])^T (\mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k] - \hat{\mathbf{x}})$$

Taking the conditional expectation $$\mathbb{E}[\cdot \mid \mathbf{Z}^k]$$ on both sides:
$$\mathbb{E}\left[ \|\mathbf{x} - \hat{\mathbf{x}}\|^2 \mid \mathbf{Z}^k \right] = \mathbb{E}\left[ \|\mathbf{x} - \mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k]\|^2 \mid \mathbf{Z}^k \right] + \|\mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k] - \hat{\mathbf{x}}\|^2 + 2 \mathbb{E}\left[ \mathbf{x} - \mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k] \mid \mathbf{Z}^k \right]^T (\mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k] - \hat{\mathbf{x}})$$

Notice the cross-term:
$$\mathbb{E}\left[ \mathbf{x} - \mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k] \mid \mathbf{Z}^k \right] = \mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k] - \mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k] = \mathbf{0}$$

Therefore:
$$\mathbb{E}\left[ \|\mathbf{x} - \hat{\mathbf{x}}\|^2 \mid \mathbf{Z}^k \right] = \mathbb{E}\left[ \|\mathbf{x} - \mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k]\|^2 \mid \mathbf{Z}^k \right] + \|\mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k] - \hat{\mathbf{x}}\|^2$$

The first term is independent of our choice of estimator $$\hat{\mathbf{x}}$$. The second term is strictly non-negative ($$\ge 0$$) and attains its unique minimum of zero if and only if:
$$\hat{\mathbf{x}}_{\text{MMSE}} = \mathbb{E}[\mathbf{x} \mid \mathbf{Z}^k]$$

Thus, the optimal MMSE state estimate is mathematically identical to the **conditional expectation** of the state vector given the accumulated measurement history.

---

### 1.4 The Orthogonality Principle in Hilbert Estimation Space

In the Hilbert space $$\mathcal{L}_2(\Omega, \mathcal{F}, \mathbb{P})$$ of square-integrable random variables equipped with inner product $$\langle \mathbf{u}, \mathbf{v} \rangle = \mathbb{E}[\mathbf{u}^T \mathbf{v}]$$, the optimal estimate $$\hat{\mathbf{x}}$$ represents the orthogonal projection of the true state $$\mathbf{x}$$ onto the subspace spanned by the observations $$\mathbf{Z}^k$$.

#### The Orthogonality Theorem:
The estimation error $$\tilde{\mathbf{x}} = \mathbf{x} - \hat{\mathbf{x}}$$ is statistically orthogonal to any linear or non-linear measurable transformation $$\mathbf{g}(\mathbf{Z}^k)$$ of the measurement data:
$$\mathbb{E}\left[ (\mathbf{x} - \hat{\mathbf{x}}) \mathbf{g}(\mathbf{Z}^k)^T \right] = \mathbf{0}_{n \times m}$$

This principle dictates that **all information contained in the observations has been completely extracted**. The residual error consists purely of unpredictable, zero-mean white noise.

---

## 2. The Singular (Linear Discrete) Kalman Filter — Complete First-Principles Derivation

### 2.1 Discrete-Time Linear State-Space Formulation

Consider a linear, time-invariant or time-varying stochastic dynamical system defined at discrete time indices $$k \in \{1, 2, 3, \dots\}$$:

$$\mathbf{x}_k = \mathbf{A}_k \mathbf{x}_{k-1} + \mathbf{B}_k \mathbf{u}_{k-1} + \mathbf{w}_{k-1}$$
$$\mathbf{z}_k = \mathbf{H}_k \mathbf{x}_k + \mathbf{v}_k$$

#### Variable and Parameter Definitions:
- $$\mathbf{x}_k \in \mathbb{R}^n$$: The hidden, true system state vector at time $$k$$.
- $$\mathbf{A}_k \in \mathbb{R}^{n \times n}$$: The state transition matrix mapping state from $$k-1$$ to $$k$$.
- $$\mathbf{u}_{k-1} \in \mathbb{R}^p$$: The known deterministic control input vector (e.g., thruster force commands).
- $$\mathbf{B}_k \in \mathbb{R}^{n \times p}$$: The control input gain matrix.
- $$\mathbf{w}_{k-1} \in \mathbb{R}^n$$: Additive Gaussian process noise representing unmodeled physical disturbances.
- $$\mathbf{z}_k \in \mathbb{R}^m$$: The observable sensor measurement vector at time $$k$$.
- $$\mathbf{H}_k \in \mathbb{R}^{m \times n}$$: The observation matrix mapping state space to measurement space.
- $$\mathbf{v}_k \in \mathbb{R}^m$$: Additive Gaussian measurement noise representing sensor electrical noise and quantization error.

#### Stochastic Noise Assumptions:
The noise processes $$\mathbf{w}_k$$ and $$\mathbf{v}_k$$ are zero-mean, mutually independent white Gaussian random sequences satisfying:
$$\mathbb{E}[\mathbf{w}_k] = \mathbf{0}, \quad \mathbb{E}[\mathbf{w}_k \mathbf{w}_j^T] = \mathbf{Q}_k \delta_{kj}$$
$$\mathbb{E}[\mathbf{v}_k] = \mathbf{0}, \quad \mathbb{E}[\mathbf{v}_k \mathbf{v}_j^T] = \mathbf{R}_k \delta_{kj}$$
$$\mathbb{E}[\mathbf{w}_k \mathbf{v}_j^T] = \mathbf{0}_{n \times m}, \quad \forall k, j$$
$$\mathbb{E}[\mathbf{w}_k \mathbf{x}_0^T] = \mathbf{0}_{n \times n}, \quad \mathbb{E}[\mathbf{v}_k \mathbf{x}_0^T] = \mathbf{0}_{m \times n}, \quad \forall k$$

where $$\delta_{kj}$$ is the Kronecker delta ($$\delta_{kj} = 1$$ if $$k = j$$, and $$0$$ otherwise), $$\mathbf{Q}_k \succeq 0$$ is the process noise covariance matrix, and $$\mathbf{R}_k \succ 0$$ is the measurement noise covariance matrix.

---

### 2.2 Step 1 Derivation: Prior State Prediction

Let $$\hat{\mathbf{x}}_{k-1|k-1}$$ denote the optimal posterior state estimate at time $$k-1$$ given all measurements up to $$k-1$$.
We define the prior (predicted) state estimate at time step $$k$$ before the measurement $$\mathbf{z}_k$$ is ingested as:
$$\hat{\mathbf{x}}_{k|k-1} = \mathbb{E}[\mathbf{x}_k \mid \mathbf{Z}^{k-1}]$$

Substitute the state dynamic equation:
$$\hat{\mathbf{x}}_{k|k-1} = \mathbb{E}[\mathbf{A}_k \mathbf{x}_{k-1} + \mathbf{B}_k \mathbf{u}_{k-1} + \mathbf{w}_{k-1} \mid \mathbf{Z}^{k-1}]$$

Using the linearity property of expectation:
$$\hat{\mathbf{x}}_{k|k-1} = \mathbf{A}_k \mathbb{E}[\mathbf{x}_{k-1} \mid \mathbf{Z}^{k-1}] + \mathbf{B}_k \mathbf{u}_{k-1} + \mathbb{E}[\mathbf{w}_{k-1} \mid \mathbf{Z}^{k-1}]$$

By definition:
$$\mathbb{E}[\mathbf{x}_{k-1} \mid \mathbf{Z}^{k-1}] = \hat{\mathbf{x}}_{k-1|k-1}$$
Since process noise $$\mathbf{w}_{k-1}$$ is zero-mean and independent of past measurements $$\mathbf{Z}^{k-1}$$:
$$\mathbb{E}[\mathbf{w}_{k-1} \mid \mathbf{Z}^{k-1}] = \mathbb{E}[\mathbf{w}_{k-1}] = \mathbf{0}$$

Thus, we obtain the exact prior state prediction equation:
$$\hat{\mathbf{x}}_{k|k-1} = \mathbf{A}_k \hat{\mathbf{x}}_{k-1|k-1} + \mathbf{B}_k \mathbf{u}_{k-1}$$

---

### 2.3 Step 2 Derivation: Prior Error Covariance Matrix

Define the prior estimation error vector $$\tilde{\mathbf{x}}_{k|k-1}$$ as:
$$\tilde{\mathbf{x}}_{k|k-1} = \mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1}$$

Substitute the true state $$\mathbf{x}_k$$ and predicted state $$\hat{\mathbf{x}}_{k|k-1}$$:
$$\tilde{\mathbf{x}}_{k|k-1} = (\mathbf{A}_k \mathbf{x}_{k-1} + \mathbf{B}_k \mathbf{u}_{k-1} + \mathbf{w}_{k-1}) - (\mathbf{A}_k \hat{\mathbf{x}}_{k-1|k-1} + \mathbf{B}_k \mathbf{u}_{k-1})$$
$$\tilde{\mathbf{x}}_{k|k-1} = \mathbf{A}_k (\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1}) + \mathbf{w}_{k-1} = \mathbf{A}_k \tilde{\mathbf{x}}_{k-1|k-1} + \mathbf{w}_{k-1}$$

The prior error covariance matrix $$\mathbf{P}_{k|k-1}$$ is defined as:
$$\mathbf{P}_{k|k-1} = \mathbb{E}\left[ \tilde{\mathbf{x}}_{k|k-1} \tilde{\mathbf{x}}_{k|k-1}^T \mid \mathbf{Z}^{k-1} \right]$$

Substitute the expanded expression for $$\tilde{\mathbf{x}}_{k|k-1}$$:
$$\mathbf{P}_{k|k-1} = \mathbb{E}\left[ (\mathbf{A}_k \tilde{\mathbf{x}}_{k-1|k-1} + \mathbf{w}_{k-1}) (\mathbf{A}_k \tilde{\mathbf{x}}_{k-1|k-1} + \mathbf{w}_{k-1})^T \right]$$
$$\mathbf{P}_{k|k-1} = \mathbb{E}\left[ \mathbf{A}_k \tilde{\mathbf{x}}_{k-1|k-1} \tilde{\mathbf{x}}_{k-1|k-1}^T \mathbf{A}_k^T + \mathbf{A}_k \tilde{\mathbf{x}}_{k-1|k-1} \mathbf{w}_{k-1}^T + \mathbf{w}_{k-1} \tilde{\mathbf{x}}_{k-1|k-1}^T \mathbf{A}_k^T + \mathbf{w}_{k-1} \mathbf{w}_{k-1}^T \right]$$

Applying expectation term by term:
$$\mathbf{P}_{k|k-1} = \mathbf{A}_k \mathbb{E}\left[\tilde{\mathbf{x}}_{k-1|k-1} \tilde{\mathbf{x}}_{k-1|k-1}^T\right] \mathbf{A}_k^T + \mathbf{A}_k \mathbb{E}\left[\tilde{\mathbf{x}}_{k-1|k-1} \mathbf{w}_{k-1}^T\right] + \mathbb{E}\left[\mathbf{w}_{k-1} \tilde{\mathbf{x}}_{k-1|k-1}^T\right] \mathbf{A}_k^T + \mathbb{E}\left[\mathbf{w}_{k-1} \mathbf{w}_{k-1}^T\right]$$

By definition:
$$\mathbb{E}\left[\tilde{\mathbf{x}}_{k-1|k-1} \tilde{\mathbf{x}}_{k-1|k-1}^T\right] = \mathbf{P}_{k-1|k-1}$$
$$\mathbb{E}\left[\mathbf{w}_{k-1} \mathbf{w}_{k-1}^T\right] = \mathbf{Q}_{k-1}$$

Because the past error $$\tilde{\mathbf{x}}_{k-1|k-1}$$ depends exclusively on noise realizations up to time $$k-2$$, it is strictly uncorrelated with the future noise $$\mathbf{w}_{k-1}$$:
$$\mathbb{E}\left[\tilde{\mathbf{x}}_{k-1|k-1} \mathbf{w}_{k-1}^T\right] = \mathbf{0}_{n \times n}$$

Thus, the middle cross-terms vanish identically, yielding the exact prior covariance propagation:
$$\mathbf{P}_{k|k-1} = \mathbf{A}_k \mathbf{P}_{k-1|k-1} \mathbf{A}_k^T + \mathbf{Q}_{k-1}$$

---

### 2.4 Step 3 Derivation: Innovation Residual and Innovation Covariance

At time $$k$$, sensor measurement $$\mathbf{z}_k$$ becomes available. The predicted measurement is:
$$\hat{\mathbf{z}}_{k|k-1} = \mathbb{E}[\mathbf{z}_k \mid \mathbf{Z}^{k-1}] = \mathbb{E}[\mathbf{H}_k \mathbf{x}_k + \mathbf{v}_k \mid \mathbf{Z}^{k-1}] = \mathbf{H}_k \hat{\mathbf{x}}_{k|k-1}$$

The **Innovation Residual Vector** $$\mathbf{y}_k \in \mathbb{R}^m$$ represents the brand new information extracted from the sensor:
$$\mathbf{y}_k = \mathbf{z}_k - \hat{\mathbf{z}}_{k|k-1} = \mathbf{z}_k - \mathbf{H}_k \hat{\mathbf{x}}_{k|k-1}$$

Substitute the measurement equation into $$\mathbf{y}_k$$:
$$\mathbf{y}_k = (\mathbf{H}_k \mathbf{x}_k + \mathbf{v}_k) - \mathbf{H}_k \hat{\mathbf{x}}_{k|k-1} = \mathbf{H}_k (\mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1}) + \mathbf{v}_k = \mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} + \mathbf{v}_k$$

The **Innovation Covariance Matrix** $$\mathbf{S}_k \in \mathbb{R}^{m \times m}$$ is defined as:
$$\mathbf{S}_k = \text{Cov}(\mathbf{y}_k) = \mathbb{E}\left[ \mathbf{y}_k \mathbf{y}_k^T \right] = \mathbb{E}\left[ (\mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} + \mathbf{v}_k) (\mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} + \mathbf{v}_k)^T \right]$$
$$\mathbf{S}_k = \mathbf{H}_k \mathbb{E}\left[\tilde{\mathbf{x}}_{k|k-1} \tilde{\mathbf{x}}_{k|k-1}^T\right] \mathbf{H}_k^T + \mathbf{H}_k \mathbb{E}\left[\tilde{\mathbf{x}}_{k|k-1} \mathbf{v}_k^T\right] + \mathbb{E}\left[\mathbf{v}_k \tilde{\mathbf{x}}_{k|k-1}^T\right] \mathbf{H}_k^T + \mathbb{E}\left[\mathbf{v}_k \mathbf{v}_k^T\right]$$

Since measurement noise $$\mathbf{v}_k$$ is uncorrelated with prior state estimation error $$\tilde{\mathbf{x}}_{k|k-1}$$:
$$\mathbb{E}\left[\tilde{\mathbf{x}}_{k|k-1} \mathbf{v}_k^T\right] = \mathbf{0}_{n \times m}$$

Therefore, we obtain the innovation covariance:
$$\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k$$

---

### 2.5 Step 4 Derivation: Posterior Update and the Joseph Form Covariance

We formulate a general linear correction structure for the posterior state estimate $$\hat{\mathbf{x}}_{k|k}$$:
$$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}_k \hat{\mathbf{x}}_{k|k-1})$$
where $$\mathbf{K}_k \in \mathbb{R}^{n \times m}$$ is an arbitrary correction gain matrix to be optimized.

Define the posterior estimation error $$\tilde{\mathbf{x}}_{k|k}$$:
$$\tilde{\mathbf{x}}_{k|k} = \mathbf{x}_k - \hat{\mathbf{x}}_{k|k}$$

Substitute the correction structure:
$$\tilde{\mathbf{x}}_{k|k} = \mathbf{x}_k - \left( \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k (\mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} + \mathbf{v}_k) \right)$$
$$\tilde{\mathbf{x}}_{k|k} = (\mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1}) - \mathbf{K}_k \mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k \mathbf{v}_k$$
$$\tilde{\mathbf{x}}_{k|k} = (\mathbf{I}_n - \mathbf{K}_k \mathbf{H}_k) \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k \mathbf{v}_k$$

Now compute the exact posterior error covariance matrix $$\mathbf{P}_{k|k} = \mathbb{E}\left[\tilde{\mathbf{x}}_{k|k} \tilde{\mathbf{x}}_{k|k}^T\right]$$:
$$\mathbf{P}_{k|k} = \mathbb{E}\left[ \left( (\mathbf{I}_n - \mathbf{K}_k \mathbf{H}_k) \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k \mathbf{v}_k \right) \left( (\mathbf{I}_n - \mathbf{K}_k \mathbf{H}_k) \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k \mathbf{v}_k \right)^T \right]$$
$$\mathbf{P}_{k|k} = (\mathbf{I}_n - \mathbf{K}_k \mathbf{H}_k) \mathbb{E}\left[\tilde{\mathbf{x}}_{k|k-1}\tilde{\mathbf{x}}_{k|k-1}^T\right] (\mathbf{I}_n - \mathbf{K}_k \mathbf{H}_k)^T - (\mathbf{I}_n - \mathbf{K}_k \mathbf{H}_k)\mathbb{E}\left[\tilde{\mathbf{x}}_{k|k-1}\mathbf{v}_k^T\right]\mathbf{K}_k^T - \mathbf{K}_k \mathbb{E}\left[\mathbf{v}_k \tilde{\mathbf{x}}_{k|k-1}^T\right](\mathbf{I}_n - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbb{E}\left[\mathbf{v}_k \mathbf{v}_k^T\right] \mathbf{K}_k^T$$

Since $$\mathbb{E}\left[\tilde{\mathbf{x}}_{k|k-1}\mathbf{v}_k^T\right] = \mathbf{0}$$, we obtain the famous **Joseph Form Covariance Equation**:
$$\mathbf{P}_{k|k} = (\mathbf{I}_n - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1} (\mathbf{I}_n - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T$$

> **Crucial Numerical Insight**: The Joseph form is algebraically valid for **any** gain $$\mathbf{K}_k$$, even sub-optimal gains. Furthermore, because it represents the sum of two quadratic forms ($$\mathbf{A}\mathbf{P}\mathbf{A}^T + \mathbf{B}\mathbf{R}\mathbf{B}^T$$), it is guaranteed by construction to be **strictly symmetric and positive semi-definite**, providing total numerical immunity against negative eigenvalues caused by finite floating-point roundoff errors.

---

### 2.6 Step 5 Derivation: Calculus of Variations & Optimal Kalman Gain

We now find the unique gain matrix $$\mathbf{K}_k$$ that minimizes the total scalar estimation variance, defined as the trace of the posterior covariance matrix:
$$J(\mathbf{K}_k) = \text{Tr}(\mathbf{P}_{k|k}) = \mathbb{E}\left[ \|\tilde{\mathbf{x}}_{k|k}\|^2 \right]$$

Expand the Joseph form equation:
$$\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1} - \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T + \mathbf{K}_k (\mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k) \mathbf{K}_k^T$$

Recall that $$\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k$$. Thus:
$$\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1} - \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T + \mathbf{K}_k \mathbf{S}_k \mathbf{K}_k^T$$

Taking the matrix trace:
$$\text{Tr}(\mathbf{P}_{k|k}) = \text{Tr}(\mathbf{P}_{k|k-1}) - \text{Tr}(\mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1}) - \text{Tr}(\mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T) + \text{Tr}(\mathbf{K}_k \mathbf{S}_k \mathbf{K}_k^T)$$

Since the trace of a transpose equals the trace of the original matrix:
$$\text{Tr}(\mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T) = \text{Tr}\left((\mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1})^T\right) = \text{Tr}(\mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1})$$

Therefore:
$$\text{Tr}(\mathbf{P}_{k|k}) = \text{Tr}(\mathbf{P}_{k|k-1}) - 2 \text{Tr}(\mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1}) + \text{Tr}(\mathbf{K}_k \mathbf{S}_k \mathbf{K}_k^T)$$

#### Matrix Calculus Identities:
For any compatible matrices $$\mathbf{X}$$, $$\mathbf{A}$$, and symmetric matrix $$\mathbf{S} = \mathbf{S}^T$$:
1. $$\frac{\partial \text{Tr}(\mathbf{X} \mathbf{A})}{\partial \mathbf{X}} = \mathbf{A}^T$$
2. $$\frac{\partial \text{Tr}(\mathbf{X} \mathbf{S} \mathbf{X}^T)}{\partial \mathbf{X}} = 2 \mathbf{X} \mathbf{S}$$

Differentiating $$\text{Tr}(\mathbf{P}_{k|k})$$ with respect to the matrix $$\mathbf{K}_k$$ and equating to the zero matrix $$\mathbf{0}_{n \times m}$$:
$$\frac{\partial \text{Tr}(\mathbf{P}_{k|k})}{\partial \mathbf{K}_k} = -2 (\mathbf{H}_k \mathbf{P}_{k|k-1})^T + 2 \mathbf{K}_k \mathbf{S}_k = \mathbf{0}_{n \times m}$$
$$-2 \mathbf{P}_{k|k-1} \mathbf{H}_k^T + 2 \mathbf{K}_k \mathbf{S}_k = \mathbf{0}_{n \times m}$$
$$\mathbf{K}_k \mathbf{S}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T$$

Post-multiplying both sides by the inverse innovation covariance $$\mathbf{S}_k^{-1}$$ (which exists because $$\mathbf{R}_k \succ 0 \implies \mathbf{S}_k \succ 0$$):
$$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{S}_k^{-1} = \mathbf{P}_{k|k-1} \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k)^{-1}$$

This completes the first-principles derivation of the **Optimal Kalman Gain Matrix**.

---

### 2.7 Algebraic Simplification of Posterior Covariance

When the optimal gain $$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{S}_k^{-1}$$ is used, post-multiply by $$\mathbf{S}_k \mathbf{K}_k^T$$:
$$\mathbf{K}_k \mathbf{S}_k \mathbf{K}_k^T = \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T$$

Substitute this into the expanded Joseph form equation:
$$\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1} - \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T + \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T$$

The last two terms cancel out perfectly:
$$\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1}$$
$$\mathbf{P}_{k|k} = (\mathbf{I}_n - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1}$$

This is the standard computationally efficient form used in real-time execution loops.

---

## 3. The Extended Kalman Filter (EKF) — Complete First-Principles Derivation

### 3.1 The Curse of Non-Linearity & Gaussian Breakdown

In general physical systems, such as an Autonomous Underwater Vehicle moving through fluid, the kinematic and dynamic equations are non-linear:
$$\mathbf{x}_k = \mathbf{f}(\mathbf{x}_{k-1}, \mathbf{u}_{k-1}) + \mathbf{w}_{k-1}$$
$$\mathbf{z}_k = \mathbf{h}(\mathbf{x}_k) + \mathbf{v}_k$$
where $$\mathbf{f}: \mathbb{R}^n \times \mathbb{R}^p \to \mathbb{R}^n$$ and $$\mathbf{h}: \mathbb{R}^n \to \mathbb{R}^m$$ are continuously differentiable ($$C^1$$) vector fields.

When a Gaussian random vector $$\mathbf{x} \sim \mathcal{N}(\boldsymbol{\mu}, \mathbf{P})$$ undergoes a non-linear mapping $$\mathbf{y} = \mathbf{g}(\mathbf{x})$$, the output PDF $$p(\mathbf{y})$$ is **no longer Gaussian**. It skews, becomes asymmetric, and can even become multi-modal. 

The Extended Kalman Filter overcomes this by performing an online **first-order multivariable Taylor series linearization** of the non-linear mappings around the current best state estimate.

---

### 3.2 Multivariable Taylor Series Expansion

Let $$\hat{\mathbf{x}}_{k-1|k-1}$$ be the optimal posterior estimate at $$k-1$$. Expand the vector field $$\mathbf{f}(\mathbf{x}_{k-1}, \mathbf{u}_{k-1})$$ around $$\hat{\mathbf{x}}_{k-1|k-1}$$:
$$\mathbf{f}(\mathbf{x}_{k-1}, \mathbf{u}_{k-1}) = \mathbf{f}(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_{k-1}) + \left. \frac{\partial \mathbf{f}}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_{k-1}} (\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1}) + \mathcal{O}\left(\|\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1}\|^2\right)$$

Neglecting second-order and higher terms $$\mathcal{O}(\|\tilde{\mathbf{x}}\|^2)$$:
$$\mathbf{f}(\mathbf{x}_{k-1}, \mathbf{u}_{k-1}) \approx \mathbf{f}(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_{k-1}) + \mathbf{F}_{k-1} \tilde{\mathbf{x}}_{k-1|k-1}$$
where $$\mathbf{F}_{k-1} \in \mathbb{R}^{n \times n}$$ is the **State Transition Jacobian Matrix**.

Similarly, expand the measurement vector field $$\mathbf{h}(\mathbf{x}_k)$$ around the predicted prior state $$\hat{\mathbf{x}}_{k|k-1}$$:
$$\mathbf{h}(\mathbf{x}_k) = \mathbf{h}(\hat{\mathbf{x}}_{k|k-1}) + \left. \frac{\partial \mathbf{h}}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k|k-1}} (\mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1}) + \mathcal{O}\left(\|\mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1}\|^2\right)$$
$$\mathbf{h}(\mathbf{x}_k) \approx \mathbf{h}(\hat{\mathbf{x}}_{k|k-1}) + \mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1}$$
where $$\mathbf{H}_k \in \mathbb{R}^{m \times n}$$ is the **Measurement Jacobian Matrix**.

---

### 3.3 Analytical Derivation of State and Measurement Jacobians

The Jacobian matrices are formally defined as the Fréchet derivative tensors:
$$\mathbf{F}_{k-1} = \left. \frac{\partial \mathbf{f}}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_{k-1}} = \begin{bmatrix}
\frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2} & \cdots & \frac{\partial f_1}{\partial x_n} \\
\frac{\partial f_2}{\partial x_1} & \frac{\partial f_2}{\partial x_2} & \cdots & \frac{\partial f_2}{\partial x_n} \\
\vdots & \vdots & \ddots & \vdots \\
\frac{\partial f_n}{\partial x_1} & \frac{\partial f_n}{\partial x_2} & \cdots & \frac{\partial f_n}{\partial x_n}
\end{bmatrix}$$

$$\mathbf{H}_k = \left. \frac{\partial \mathbf{h}}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k|k-1}} = \begin{bmatrix}
\frac{\partial h_1}{\partial x_1} & \frac{\partial h_1}{\partial x_2} & \cdots & \frac{\partial h_1}{\partial x_n} \\
\frac{\partial h_2}{\partial x_1} & \frac{\partial h_2}{\partial x_2} & \cdots & \frac{\partial h_2}{\partial x_n} \\
\vdots & \vdots & \ddots & \vdots \\
\frac{\partial h_m}{\partial x_1} & \frac{\partial h_m}{\partial x_2} & \cdots & \frac{\partial h_m}{\partial x_n}
\end{bmatrix}$$

---

### 3.4 The Discrete EKF Predict-Correct Recursive Equations

Substituting the Taylor expansions into the optimal estimation framework yields the complete recursive EKF cycle:

#### Phase 1: Non-Linear State & Covariance Prediction
1. **Prior State Vector Propagation** (propagated through the exact, unabridged non-linear function):
   $$\hat{\mathbf{x}}_{k|k-1} = \mathbf{f}(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_{k-1})$$
2. **Prior Error Covariance Propagation** (propagated via the Jacobian):
   $$\mathbf{P}_{k|k-1} = \mathbf{F}_{k-1} \mathbf{P}_{k-1|k-1} \mathbf{F}_{k-1}^T + \mathbf{Q}_{k-1}$$

#### Phase 2: Measurement Innovation & Posterior Update
3. **Innovation Residual Vector**:
   $$\mathbf{y}_k = \mathbf{z}_k - \mathbf{h}(\hat{\mathbf{x}}_{k|k-1})$$
4. **Innovation Covariance Matrix**:
   $$\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k$$
5. **Near-Optimal Kalman Gain**:
   $$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{S}_k^{-1}$$
6. **Posterior State Correction**:
   $$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k$$
7. **Posterior Covariance Update**:
   $$\mathbf{P}_{k|k} = (\mathbf{I}_n - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1}$$

---

### 3.5 Continuous-Discrete Extended Kalman Filter (C-D EKF) and Differential Riccati Integration

In marine robotics, physical vehicle dynamics evolve in continuous time according to differential equations, whereas digital microcontrollers sample sensors at discrete intervals $$\Delta t$$.

Let the continuous physical plant be governed by:
$$\dot{\mathbf{x}}(t) = \mathbf{f}_c(\mathbf{x}(t), \mathbf{u}(t)) + \mathbf{G}_c(t) \mathbf{w}_c(t)$$
where $$\mathbf{w}_c(t)$$ is continuous zero-mean Gaussian white noise with power spectral density matrix $$\mathbf{S}_w$$:
$$\mathbb{E}[\mathbf{w}_c(t) \mathbf{w}_c(\tau)^T] = \mathbf{S}_w \delta(t - \tau)$$

Between measurement arrivals ($$t \in [t_{k-1}, t_k]$$), the continuous conditional state mean and error covariance satisfy the coupled non-linear differential equations:
$$\dot{\hat{\mathbf{x}}}(t) = \mathbf{f}_c(\hat{\mathbf{x}}(t), \mathbf{u}(t))$$
$$\dot{\mathbf{P}}(t) = \mathbf{F}_c(t) \mathbf{P}(t) + \mathbf{P}(t) \mathbf{F}_c(t)^T + \mathbf{G}_c(t) \mathbf{S}_w \mathbf{G}_c(t)^T$$
where $$\mathbf{F}_c(t) = \left. \frac{\partial \mathbf{f}_c}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}(t)}$$.

Integrating these equations over interval $$\Delta t = t_k - t_{k-1}$$ using a 4th-order Runge-Kutta (RK4) integrator or first-order matrix exponential expansion gives the exact discrete prior propagation used in our subsea companion computer.

---

# Part II: Comprehensive Derivation of the AUV Kalman Filter Suite

```
                            ETHERNET TETHER (192.168.2.x)
     TOPSIDE WORKSTATION (LAPTOP)                 │             SUBSEA VEHICLE (AUV HULL)
┌────────────────────────────────────────────┐    │    ┌─────────────────────────────────────────┐
│ • RTSP H.264 Video Ingestion (50-60 FPS)   │◄───┼────│ • RPi 4B: BlueOS System & Camera Server │
│ • YOLO26 World Neural Detection (RTX GPU)  │    │    │ • AUVDynamicsKalmanFilter (4-DOF EKF)   │
│ • AUVVisualKalmanFilter (8D Position/Scale)│    │    │   Estimates [u, v, w, r] & currents     │
│ • Visual Servoing Guidance Law             │────┼───►│ • Autonomous Tracking Setpoints         │
└────────────────────────────────────────────┘    │    └────────────────────┬────────────────────┘
                                                  │                         │ USB MAVLink (/dev/ttyACM0)
                                                  │                         ▼
                                                  │    ┌─────────────────────────────────────────┐
                                                  │    │ • Pixhawk 2.4.8: Stock ArduSub Firmware │
                                                  │    │   400 Hz EKF3 IMU/Depth Attitude PID    │
                                                  │    │   6x T200 PWM Motor Mixing              │
                                                  └────┴─────────────────────────────────────────┘
```

---

## 4. Topside Visual Target Kalman Filter (`AUVVisualKalmanFilter`)

The visual target tracking filter runs on the topside laptop workstation inside [`auv_yolo_tracking.py`](file:///home/radhi/Documents/AUV_GitHub_Upload/auv_yolo_tracking.py), ingesting bounding boxes detected by the YOLO26 World neural network.

### 4.1 Pinhole Camera Geometry & Perspective Coordinate Projection

Let the subsea camera coordinate frame be attached to the optical center, with $$Z_c$$ along the optical axis, $$X_c$$ pointing right, and $$Y_c$$ pointing down.
Under the ideal pinhole camera model, a 3D target point $$\mathbf{P}_c = [X_c, Y_c, Z_c]^T$$ projects onto the 2D digital image sensor coordinates $$(x_p, y_p)$$ in pixels via perspective division:
$$x_p = f_x \frac{X_c}{Z_c} + c_x$$
$$y_p = f_y \frac{Y_c}{Z_c} + c_y$$
where $$f_x, f_y$$ are the camera focal lengths in pixel units, and $$(c_x, c_y)$$ is the principal point (image center).

Similarly, a physical target of metric width $$W_{\text{target}}$$ and height $$H_{\text{target}}$$ at range $$Z_c$$ projects a 2D bounding box with pixel width $$w$$ and pixel height $$h$$:
$$w = f_x \frac{W_{\text{target}}}{Z_c}$$
$$h = f_y \frac{H_{\text{target}}}{Z_c}$$

---

### 4.2 8D State-Space Vector Formulation

To track horizontal and vertical target motions simultaneously with forward surge range changes, the state vector is formulated in an **8-dimensional Cartesian-Scale space**:
$$\mathbf{x}_{\text{vis}} = \begin{bmatrix} x & y & w & h & v_x & v_y & v_w & v_h \end{bmatrix}^T \in \mathbb{R}^8$$

#### Physical Meaning of Every Component:
1. $$x$$: Pixel horizontal centroid coordinate on the camera image plane ($$[0, W_{\text{frame}}]$$, pixels).
2. $$y$$: Pixel vertical centroid coordinate on the camera image plane ($$[0, H_{\text{frame}}]$$, pixels).
3. $$w$$: Projected target bounding box width (pixels).
4. $$h$$: Projected target bounding box height (pixels).
5. $$v_x = \dot{x}$$: Apparent horizontal image-plane velocity (pixels/second).
6. $$v_y = \dot{y}$$: Apparent vertical image-plane velocity (pixels/second).
7. $$v_w = \dot{w}$$: Expansion or contraction rate of the bounding box width (pixels/second).
8. $$v_h = \dot{h}$$: Expansion or contraction rate of the bounding box height (pixels/second).

---

### 4.3 Continuous White Noise Acceleration (CWNA) Derivation

Between frames, the target and vehicle kinematics are modeled as a **Continuous White Noise Acceleration (CWNA)** process. We assume target acceleration is driven by continuous zero-mean white noise:
$$\ddot{x}(t) = w_x(t), \quad \ddot{y}(t) = w_y(t), \quad \ddot{w}(t) = w_w(t), \quad \ddot{h}(t) = w_h(t)$$

Let $$\mathbf{p}(t) = [x(t), y(t), w(t), h(t)]^T$$ and $$\mathbf{v}(t) = [v_x(t), v_y(t), v_w(t), v_h(t)]^T$$. The continuous state equation is:
$$\frac{d}{dt} \begin{bmatrix} \mathbf{p}(t) \\ \mathbf{v}(t) \end{bmatrix} = \begin{bmatrix} \mathbf{0}_{4\times 4} & \mathbf{I}_{4\times 4} \\ \mathbf{0}_{4\times 4} & \mathbf{0}_{4\times 4} \end{bmatrix} \begin{bmatrix} \mathbf{p}(t) \\ \mathbf{v}(t) \end{bmatrix} + \begin{bmatrix} \mathbf{0}_{4\times 4} \\ \mathbf{I}_{4\times 4} \end{bmatrix} \mathbf{w}(t)$$

In compact notation:
$$\dot{\mathbf{x}}_{\text{vis}}(t) = \mathbf{A}_c \mathbf{x}_{\text{vis}}(t) + \mathbf{G}_c \mathbf{w}(t)$$
where:
$$\mathbf{A}_c = \begin{bmatrix} \mathbf{0}_{4\times 4} & \mathbf{I}_{4\times 4} \\ \mathbf{0}_{4\times 4} & \mathbf{0}_{4\times 4} \end{bmatrix} \in \mathbb{R}^{8 \times 8}, \quad \mathbf{G}_c = \begin{bmatrix} \mathbf{0}_{4\times 4} \\ \mathbf{I}_{4\times 4} \end{bmatrix} \in \mathbb{R}^{8 \times 4}$$

The continuous noise covariance is:
$$\mathbb{E}[\mathbf{w}(t) \mathbf{w}(\tau)^T] = \mathbf{S}_w \delta(t - \tau) = q_s \mathbf{I}_{4\times 4} \delta(t - \tau)$$
where $$q_s > 0$$ is the continuous acceleration power spectral density ($$\text{px}^2/\text{s}^3$$).

---

### 4.4 Exact Discretization of State Transition Matrix A(Δt)

The exact discrete state transition matrix $$\mathbf{A}(\Delta t)$$ is obtained via the matrix exponential:
$$\mathbf{A}(\Delta t) = e^{\mathbf{A}_c \Delta t} = \sum_{k=0}^{\infty} \frac{(\mathbf{A}_c \Delta t)^k}{k!} = \mathbf{I}_8 + \mathbf{A}_c \Delta t + \frac{1}{2!} \mathbf{A}_c^2 \Delta t^2 + \dots$$

Compute higher powers of $$\mathbf{A}_c$$:
$$\mathbf{A}_c^2 = \begin{bmatrix} \mathbf{0} & \mathbf{I} \\ \mathbf{0} & \mathbf{0} \end{bmatrix} \begin{bmatrix} \mathbf{0} & \mathbf{I} \\ \mathbf{0} & \mathbf{0} \end{bmatrix} = \begin{bmatrix} \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} \end{bmatrix} = \mathbf{0}_{8 \times 8}$$

Because $$\mathbf{A}_c$$ is nilpotent of degree 2 ($$\mathbf{A}_c^2 = \mathbf{0}$$), the infinite series terminates after exactly two terms:
$$\mathbf{A}(\Delta t) = \mathbf{I}_8 + \mathbf{A}_c \Delta t = \begin{bmatrix} \mathbf{I}_{4\times 4} & \Delta t \mathbf{I}_{4\times 4} \\ \mathbf{0}_{4\times 4} & \mathbf{I}_{4\times 4} \end{bmatrix}$$

In full 8x8 scalar expansion:
$$\mathbf{A}(\Delta t) = \begin{bmatrix}
1 & 0 & 0 & 0 & \Delta t & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & \Delta t & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 & \Delta t & 0 \\
0 & 0 & 0 & 1 & 0 & 0 & 0 & \Delta t \\
0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}$$

---

### 4.5 Exact Discretization of Process Noise Covariance Q(Δt) via Matrix Exponential Integrals

The discrete process noise covariance matrix $$\mathbf{Q}_k$$ is derived rigorously from the continuous spectral density by integrating over time interval $$\Delta t$$:
$$\mathbf{Q}_k = \int_0^{\Delta t} e^{\mathbf{A}_c (\Delta t - \tau)} \mathbf{G}_c \mathbf{S}_w \mathbf{G}_c^T \left( e^{\mathbf{A}_c (\Delta t - \tau)} \right)^T d\tau$$

Let substitution variable $$\lambda = \Delta t - \tau$$. As $$\tau$$ ranges from $$0$$ to $$\Delta t$$, $$\lambda$$ ranges from $$\Delta t$$ to $$0$$, and $$d\tau = -d\lambda$$:
$$\mathbf{Q}_k = \int_0^{\Delta t} e^{\mathbf{A}_c \lambda} \mathbf{G}_c \mathbf{S}_w \mathbf{G}_c^T \left( e^{\mathbf{A}_c \lambda} \right)^T d\lambda$$

Evaluate the integrand kernel:
$$e^{\mathbf{A}_c \lambda} \mathbf{G}_c = \begin{bmatrix} \mathbf{I}_{4\times 4} & \lambda \mathbf{I}_{4\times 4} \\ \mathbf{0}_{4\times 4} & \mathbf{I}_{4\times 4} \end{bmatrix} \begin{bmatrix} \mathbf{0}_{4\times 4} \\ \mathbf{I}_{4\times 4} \end{bmatrix} = \begin{bmatrix} \lambda \mathbf{I}_{4\times 4} \\ \mathbf{I}_{4\times 4} \end{bmatrix}$$

Multiplying by $$\mathbf{S}_w = q_s \mathbf{I}_{4\times 4}$$ and the transpose:
$$e^{\mathbf{A}_c \lambda} \mathbf{G}_c \mathbf{S}_w \mathbf{G}_c^T \left( e^{\mathbf{A}_c \lambda} \right)^T = q_s \begin{bmatrix} \lambda \mathbf{I}_{4\times 4} \\ \mathbf{I}_{4\times 4} \end{bmatrix} \begin{bmatrix} \lambda \mathbf{I}_{4\times 4} & \mathbf{I}_{4\times 4} \end{bmatrix} = q_s \begin{bmatrix} \lambda^2 \mathbf{I}_{4\times 4} & \lambda \mathbf{I}_{4\times 4} \\ \lambda \mathbf{I}_{4\times 4} & \mathbf{I}_{4\times 4} \end{bmatrix}$$

Now compute the definite integral term by term:
$$\int_0^{\Delta t} \lambda^2 d\lambda = \left[ \frac{\lambda^3}{3} \right]_0^{\Delta t} = \frac{\Delta t^3}{3}$$
$$\int_0^{\Delta t} \lambda \, d\lambda = \left[ \frac{\lambda^2}{2} \right]_0^{\Delta t} = \frac{\Delta t^2}{2}$$
$$\int_0^{\Delta t} 1 \, d\lambda = \left[ \lambda \right]_0^{\Delta t} = \Delta t$$

Therefore, the exact discrete process noise covariance is:
$$\mathbf{Q}_k = q_s \begin{bmatrix} \frac{\Delta t^3}{3} \mathbf{I}_{4\times 4} & \frac{\Delta t^2}{2} \mathbf{I}_{4\times 4} \\ \frac{\Delta t^2}{2} \mathbf{I}_{4\times 4} & \Delta t \mathbf{I}_{4\times 4} \end{bmatrix} \in \mathbb{R}^{8 \times 8}$$

In our implementation, baseline parameter $$q_s = 0.05$$. Notice that the off-diagonal block $$\frac{\Delta t^2}{2} \mathbf{I}$$ explicitly models the physical cross-correlation between position uncertainty and velocity perturbations!

---

### 4.6 Analytical Monocular Scale Rate & Surge Range-Rate Derivation

Standard monocular cameras cannot directly measure physical depth ($$Z_c$$). However, our 8D Kalman filter estimates bounding box growth rates ($$v_w, v_h$$), which enables closed-loop distance control without an expensive acoustic Doppler Velocity Log (DVL).

Let the projected bounding box area be defined as:
$$\mathcal{A} = w \cdot h$$

Taking the total derivative with respect to time using the product rule:
$$\frac{d\mathcal{A}}{dt} = \dot{w} h + w \dot{h} = v_w h + w v_h$$

Now relate projected area $$\mathcal{A}$$ to physical metric target distance $$Z_c$$:
$$w = f_x \frac{W}{Z_c}, \quad h = f_y \frac{H}{Z_c} \implies \mathcal{A} = f_x f_y \frac{W H}{Z_c^2}$$

Differentiate $$\mathcal{A}$$ with respect to $$Z_c$$:
$$\frac{d\mathcal{A}}{dt} = \frac{\partial \mathcal{A}}{\partial Z_c} \frac{dZ_c}{dt} = \left( -2 f_x f_y \frac{W H}{Z_c^3} \right) \dot{Z}_c = -2 \frac{\mathcal{A}}{Z_c} \dot{Z}_c$$

Divide by $$\mathcal{A}$$:
$$\frac{1}{\mathcal{A}} \frac{d\mathcal{A}}{dt} = -2 \frac{\dot{Z}_c}{Z_c}$$

Solving for the physical forward surge approach velocity $$\dot{Z}_c$$:
$$\dot{Z}_c = -\frac{Z_c}{2} \left( \frac{v_w h + w v_h}{w \cdot h} \right) = -\frac{Z_c}{2} \left( \frac{v_w}{w} + \frac{v_h}{h} \right)$$

This provides the exact mathematical proof: **the relative expansion rate of the bounding box is directly proportional to the vehicle's forward approach speed toward the target**.
In [`AUVVisualKalmanFilter.get_scale_rates()`](file:///home/radhi/Documents/AUV_GitHub_Upload/kalman_filter.py#L251-L261), this expansion rate $$\dot{\mathcal{A}}$$ is computed in 4 nanoseconds and dispatched to the AUV surge controller to automatically maintain standoff distance!

---

### 4.7 Adaptive Confidence-Weighted Measurement Covariance R(conf)

In underwater environments, backscatter, bubbles, and suspended sediment degrade optical clarity. Fixed measurement covariance $$\mathbf{R}$$ fails: it either over-filters sharp images or tracks false ghost detections in muddy water.

To solve this, our filter dynamically scales measurement covariance $$\mathbf{R}_k$$ at every frame using the neural detection confidence $$\text{conf} \in [0.15, 1.0]$$:
$$\mathbf{R}(\text{conf}) = \frac{\mathbf{R}_0}{\max(\text{conf}, 0.15)^2}$$
where baseline covariance:
$$\mathbf{R}_0 = \text{diag}\left[ \sigma_x^2, \sigma_y^2, \sigma_w^2, \sigma_h^2 \right] = \text{diag}[0.20, 0.20, 0.50, 0.50]$$

#### Behavioral Effect:
- When YOLO detects an object with high certainty ($$\text{conf} = 0.95$$):
  $$\mathbf{R} \approx \frac{\mathbf{R}_0}{0.90} \approx 1.11 \mathbf{R}_0 \implies \mathbf{K} \approx \mathbf{I} \implies \text{Trust measurement immediately}$$
- When water gets murky and confidence drops ($$\text{conf} = 0.30$$):
  $$\mathbf{R} = \frac{\mathbf{R}_0}{0.09} = 11.11 \mathbf{R}_0 \implies \mathbf{K} \to \mathbf{0} \implies \text{Rely on Kalman motion model}$$

---

### 4.8 Mahalanobis Distance Innovation Outlier Gating

To reject false positive detections caused by sunlight glints on the water surface or sudden bubble clouds, we apply statistical **Innovation Gating**:
$$d_M^2 = \mathbf{y}_k^T \mathbf{S}_k^{-1} \mathbf{y}_k \le \gamma$$
where $$d_M^2$$ is the squared Mahalanobis distance, which follows a Chi-Square distribution with $$m = 4$$ degrees of freedom ($$\chi_4^2$$). 

In [`AUVVisualKalmanFilter.update()`](file:///home/radhi/Documents/AUV_GitHub_Upload/kalman_filter.py#L183-L188), we apply an Euclidean innovation distance threshold of $$\gamma_{\text{px}} = 300\text{ px}$$. If a candidate detection jumps across the screen by more than 300 pixels in a single 33 ms frame, it is flagged as physically impossible for an underwater target, rejected, and routed to the dead-reckoning predictor!

---

### 4.9 Occlusion Bridging & Dead-Reckoning Mathematical Mechanics

When a target is temporarily obscured behind a structure or bubble cloud for $$N_{\text{missed}}$$ consecutive frames:
1. The measurement update is bypassed entirely ($$\mathbf{K}_k = \mathbf{0}$$).
2. The filter executes pure state propagation:
   $$\hat{\mathbf{x}}_{k|k-1} = \mathbf{A}(\Delta t) \hat{\mathbf{x}}_{k-1|k-1}$$
   $$\mathbf{P}_{k|k-1} = \mathbf{A}(\Delta t) \mathbf{P}_{k-1|k-1} \mathbf{A}^T(\Delta t) + \mathbf{Q}_k$$
3. Target position propagates forward linearly using its estimated velocity:
   $$\hat{x}_k = \hat{x}_{k-1} + \hat{v}_x \Delta t$$
   $$\hat{y}_k = \hat{y}_{k-1} + \hat{v}_y \Delta t$$
4. Error covariance $$\mathbf{P}$$ grows monotonically with each step ($$+ \mathbf{Q}$$), reflecting expanding uncertainty.
5. If $$N_{\text{missed}} \le 15$$ frames (~0.5 seconds), tracking continuity is preserved with zero thruster dropout. If $$N_{\text{missed}} > 15$$, the target is safely declared lost.

---

## 5. Subsea Hydrodynamic Dynamics Extended Kalman Filter (`AUVDynamicsKalmanFilter`)

The dynamics filter runs on the subsea Raspberry Pi 4B under BlueOS, fusing Pixhawk IMU/depth telemetry and motor thrust commands to estimate true surge/sway/heave velocities and isolate external ocean current forces.

### 5.1 SNAME Coordinate Frames & Kinematic Reductions

Following the Society of Naval Architects and Marine Engineers (SNAME) 1950 notation standardized by Fossen (2021):
- **Earth-Fixed Inertial Frame** $$\{n\}$$: North-East-Down (NED). Origin fixed on the sea surface.
- **Body-Fixed Vehicle Frame** $$\{b\}$$: Moving coordinate frame with origin at the vehicle's center of gravity (CG). $$x_b$$ forward (bow), $$y_b$$ starboard, $$z_b$$ downward.

The kinematic velocity vector is:
$$\boldsymbol{\nu} = \begin{bmatrix} u & v & w & p & q & r \end{bmatrix}^T$$
where:
- $$u$$: Surge linear velocity along $$x_b$$ (m/s)
- $$v$$: Sway linear velocity along $$y_b$$ (m/s)
- $$w$$: Heave linear velocity along $$z_b$$ (m/s)
- $$p$$: Roll angular rate about $$x_b$$ (rad/s)
- $$q$$: Pitch angular rate about $$y_b$$ (rad/s)
- $$r$$: Yaw angular rate about $$z_b$$ (rad/s)

---

### 5.2 Fossen's 6-DOF Hydrodynamic Kinetics Equations

The comprehensive 6-DOF non-linear equations of motion for an underwater vehicle are:
$$\mathbf{M}_{RB} \dot{\boldsymbol{\nu}} + \mathbf{C}_{RB}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{M}_A \dot{\boldsymbol{\nu}} + \mathbf{C}_A(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{D}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{g}(\boldsymbol{\eta}) = \boldsymbol{\tau} + \boldsymbol{\tau}_{\text{current}}$$

Group total mass and total Coriolis terms:
$$\mathbf{M} = \mathbf{M}_{RB} + \mathbf{M}_A$$
$$\mathbf{C}(\boldsymbol{\nu}) = \mathbf{C}_{RB}(\boldsymbol{\nu}) + \mathbf{C}_A(\boldsymbol{\nu})$$

yielding:
$$\mathbf{M} \dot{\boldsymbol{\nu}} + \mathbf{C}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{D}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{g}(\boldsymbol{\eta}) = \boldsymbol{\tau} + \boldsymbol{\tau}_{\text{dist}}$$

---

### 5.3 Variable-by-Variable 4-DOF Decoupled Reduction

For our custom 5-DOF AUV frame (and BlueROV2 Standard):
1. **Metacentric Restoring Stability**: The center of buoyancy (CB) is located $$10\text{ cm}$$ directly above the center of gravity (CG) ($$\overline{BG} = z_g - z_b = 0.05\text{ m}$$). This creates a massive static righting moment in roll ($$\phi$$) and pitch ($$\theta$$):
   $$K_{\text{restoring}} = -\rho g \nabla \overline{BG} \sin\phi \approx 0 \implies \phi \approx 0, \quad p \approx 0$$
   $$M_{\text{restoring}} = -\rho g \nabla \overline{BG} \sin\theta \approx 0 \implies \theta \approx 0, \quad q \approx 0$$
2. Therefore, roll ($$p$$) and pitch ($$q$$) decouple passively, reducing the operational dynamic degrees of freedom to **4-DOF**:
   $$\boldsymbol{\nu}_{\text{4DOF}} = \begin{bmatrix} u & v & w & r \end{bmatrix}^T$$
   representing Surge, Sway, Heave, and Yaw rate.

---

### 5.4 Generalized Inertia Matrix M: Rigid Body & Hydrodynamic Added Mass

When an underwater body accelerates, it must physically displace a volume of surrounding fluid. This induces a reaction force proportional to acceleration, modeled as the **Hydrodynamic Added Mass Matrix** $$\mathbf{M}_A$$.

Total system inertia is:
$$\mathbf{M} = \mathbf{M}_{RB} + \mathbf{M}_A$$

For a symmetrical hull at low to moderate speeds, cross-coupling terms are negligible, giving a diagonal generalized inertia matrix:
$$\mathbf{M} = \text{diag}\left[ m - X_{\dot{u}}, \, m - Y_{\dot{v}}, \, m - Z_{\dot{w}}, \, I_z - N_{\dot{r}} \right]$$

#### Exact Hydrodynamic Values for Our Vehicle:
- Rigid-body mass: $$m = 11.5\text{ kg}$$
- Yaw rotational inertia: $$I_z = 0.16\text{ kg}\cdot\text{m}^2$$
- Added mass in surge ($$X_{\dot{u}}$$, derived via strip theory for rectangular box): $$-6.36\text{ kg}$$
- Added mass in sway ($$Y_{\dot{v}}$$): $$-7.12\text{ kg}$$
- Added mass in heave ($$Z_{\dot{w}}$$): $$-18.68\text{ kg}$$
- Added mass moment of inertia in yaw ($$N_{\dot{r}}$$): $$-0.09\text{ kg}\cdot\text{m}^2$$

Computing the entries of $$\mathbf{M}$$:
$$M_u = m - X_{\dot{u}} = 11.5 - (-6.36) = 17.86\text{ kg}$$
$$M_v = m - Y_{\dot{v}} = 11.5 - (-7.12) = 18.62\text{ kg}$$
$$M_w = m - Z_{\dot{w}} = 11.5 - (-18.68) = 30.18\text{ kg}$$
$$M_r = I_z - N_{\dot{r}} = 0.16 - (-0.09) = 0.25\text{ kg}\cdot\text{m}^2$$

$$\mathbf{M} = \text{diag}[17.86, 18.62, 30.18, 0.25]$$

Notice that **effective heave inertia is nearly triple the rigid-body mass** due to water entrainment above and below the flat hull surfaces!

---

### 5.5 Hydrodynamic Damping Matrix D(ν): Linear Skin Friction & Non-Linear Quadratic Form Drag

Hydrodynamic damping in water consists of two distinct physical mechanisms:
$$\mathbf{D}(\boldsymbol{\nu})\boldsymbol{\nu} = \mathbf{D}_{\text{lin}}\boldsymbol{\nu} + \mathbf{D}_{\text{quad}}|\boldsymbol{\nu}|\boldsymbol{\nu}$$
where:
- **Linear Damping** ($$\mathbf{D}_{\text{lin}}$$) represents laminar skin friction boundary layer shearing at low speeds ($$< 0.1\text{ m/s}$$).
- **Quadratic Damping** ($$\mathbf{D}_{\text{quad}}$$ represents turbulent vortex shedding and form drag:
  $$F_{\text{drag}} = \frac{1}{2} \rho C_d A_{\text{proj}} |u|u$$

#### Coefficient Vectors:
$$\mathbf{D}_{\text{lin}} = \begin{bmatrix} X_u \\ Y_v \\ Z_w \\ N_r \end{bmatrix} = \begin{bmatrix} 13.7\text{ Ns/m} \\ 0.0\text{ Ns/m} \\ 33.8\text{ Ns/m} \\ 0.0\text{ Nms/rad} \end{bmatrix}$$
$$\mathbf{D}_{\text{quad}} = \begin{bmatrix} X_{u|u|} \\ Y_{v|v|} \\ Z_{w|w|} \\ N_{r|r|} \end{bmatrix} = \begin{bmatrix} 141.0\text{ Ns}^2/\text{m}^2 \\ 217.0\text{ Ns}^2/\text{m}^2 \\ 190.0\text{ Ns}^2/\text{m}^2 \\ 1.5\text{ Nms}^2/\text{rad}^2 \end{bmatrix}$$

---

### 5.6 Ocean Current Disturbance Observer Formulation

Subsea ocean currents exert external drag forces that cause stationary vehicles to drift. Rather than treating current forces as unmodeled noise, our EKF augments the state vector with an **Integral Disturbance Observer**:
$$\mathbf{x}_{\text{dyn}} = \begin{bmatrix} u & v & w & r & d_u & d_v \end{bmatrix}^T \in \mathbb{R}^6$$
where $$d_u$$ and $$d_v$$ are the unknown external environmental forces (in Newtons) acting along the surge and sway axes.

Current disturbances vary slowly relative to thruster dynamics and are modeled as a **first-order Gauss-Markov random walk process**:
$$\dot{d}_u = -\frac{1}{T_c} d_u + w_{du}, \quad \dot{d}_v = -\frac{1}{T_c} d_v + w_{dv}$$
where $$T_c \approx 50\text{ s}$$ is the correlation time constant, and $$w_{du}, w_{dv}$$ are zero-mean white Gaussian driving noises.

---

### 5.7 First-Principles Derivation of the Analytical 6x6 Continuous Jacobian Matrix F

The continuous non-linear differential state equations $$\dot{\mathbf{x}} = \mathbf{f}_c(\mathbf{x}, \boldsymbol{\tau})$$ are:
$$\dot{u} = \frac{\tau_u - (X_u u + X_{u|u|} |u|u) + d_u}{M_u}$$
$$\dot{v} = \frac{\tau_v - (Y_v v + Y_{v|v|} |v|v) + d_v}{M_v}$$
$$\dot{w} = \frac{\tau_w - (Z_w w + Z_{w|w|} |w|w)}{M_w}$$
$$\dot{r} = \frac{\tau_r - (N_r r + N_{r|r|} |r|r)}{M_r}$$
$$\dot{d}_u = 0$$
$$\dot{d}_v = 0$$

Now evaluate the partial derivatives to construct the continuous Jacobian $$\mathbf{F}_c = \frac{\partial \mathbf{f}_c}{\partial \mathbf{x}}$$:

#### Derivative with respect to velocity $$u$$:
Recall that for any scalar $$u$$, $$\frac{d}{du}(|u|u) = \frac{d}{du}(u \cdot \text{sgn}(u) u) = 2 |u|$$.
Therefore:
$$\frac{\partial \dot{u}}{\partial u} = -\frac{X_u + 2 X_{u|u|} |u|}{M_u}$$
$$\frac{\partial \dot{u}}{\partial d_u} = \frac{1}{M_u}$$

#### Derivative with respect to velocity $$v$$:
$$\frac{\partial \dot{v}}{\partial v} = -\frac{Y_v + 2 Y_{v|v|} |v|}{M_v}$$
$$\frac{\partial \dot{v}}{\partial d_v} = \frac{1}{M_v}$$

#### Derivative with respect to velocity $$w$$:
$$\frac{\partial \dot{w}}{\partial w} = -\frac{Z_w + 2 Z_{w|w|} |w|}{M_w}$$

#### Derivative with respect to yaw rate $$r$$:
$$\frac{\partial \dot{r}}{\partial r} = -\frac{N_r + 2 N_{r|r|} |r|}{M_r}$$

All cross-derivatives $$\frac{\partial \dot{u}}{\partial v}, \frac{\partial \dot{u}}{\partial w}$$, etc., are zero due to symmetrical decoupling.

Thus, the exact continuous Jacobian matrix $$\mathbf{F}_c \in \mathbb{R}^{6 \times 6}$$ is:
$$\mathbf{F}_c = \begin{bmatrix}
-\frac{X_u + 2 X_{u|u|} |u|}{M_u} & 0 & 0 & 0 & \frac{1}{M_u} & 0 \\
0 & -\frac{Y_v + 2 Y_{v|v|} |v|}{M_v} & 0 & 0 & 0 & \frac{1}{M_v} \\
0 & 0 & -\frac{Z_w + 2 Z_{w|w|} |w|}{M_w} & 0 & 0 & 0 \\
0 & 0 & 0 & -\frac{N_r + 2 N_{r|r|} |r|}{M_r} & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}$$

---

### 5.8 Cayley-Hamilton Discretization Φ = I + FΔt

To propagate error covariance across discrete sample step $$\Delta t = 0.02\text{ s}$$ (50 Hz), we compute the discrete transition matrix $$\boldsymbol{\Phi} = e^{\mathbf{F}_c \Delta t}$$.
Using first-order Taylor series truncation:
$$\boldsymbol{\Phi} \approx \mathbf{I}_6 + \mathbf{F}_c \Delta t$$

In full matrix expansion:
$$\boldsymbol{\Phi} = \begin{bmatrix}
1 - \frac{X_u + 2 X_{u|u|} |u|}{M_u}\Delta t & 0 & 0 & 0 & \frac{\Delta t}{M_u} & 0 \\
0 & 1 - \frac{Y_v + 2 Y_{v|v|} |v|}{M_v}\Delta t & 0 & 0 & 0 & \frac{\Delta t}{M_v} \\
0 & 0 & 1 - \frac{Z_w + 2 Z_{w|w|} |w|}{M_w}\Delta t & 0 & 0 & 0 \\
0 & 0 & 0 & 1 - \frac{N_r + 2 N_{r|r|} |r|}{M_r}\Delta t & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}$$

Error covariance is propagated via:
$$\mathbf{P}_{k|k-1} = \boldsymbol{\Phi} \mathbf{P}_{k-1|k-1} \boldsymbol{\Phi}^T + \mathbf{Q}_{\text{dyn}}$$

where process noise covariance $$\mathbf{Q}_{\text{dyn}} = \text{diag}[0.002, 0.002, 0.002, 0.001, 0.05, 0.05] \times \Delta t$$.

---

### 5.9 Multi-Sensor Innovation & Update on Raspberry Pi 4B

At each step, observation vector $$\mathbf{z}_k = [u_{\text{meas}}, v_{\text{meas}}, w_{\text{meas}}, r_{\text{meas}}]^T \in \mathbb{R}^4$$ is constructed by fusing:
1. Integrated linear accelerations from the Pixhawk 2.4.8 ICM-20608 IMU ($$u_m, v_m$$).
2. Differentiated barometric water pressure from the MS5837-30BA subsea depth sensor ($$w_m = \frac{d(\text{depth})}{dt}$$).
3. Gyroscopic angular rate from the Pixhawk 3-axis gyro ($$r_m$$).

The observation matrix is:
$$\mathbf{H} = \begin{bmatrix} \mathbf{I}_{4\times 4} & \mathbf{0}_{4\times 2} \end{bmatrix} \in \mathbb{R}^{4 \times 6}$$

The filter executes the standard correction:
$$\mathbf{y}_k = \mathbf{z}_k - \mathbf{H}\hat{\mathbf{x}}_{k|k-1}$$
$$\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}_{\text{dyn}}$$
$$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T \mathbf{S}_k^{-1}$$
$$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k$$
$$\mathbf{P}_{k|k} = (\mathbf{I}_6 - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}$$

As the vehicle moves, any persistent difference between commanded thruster force $$\tau$$ and measured acceleration is absorbed by state variables $$\hat{d}_u, \hat{d}_v$$, providing instantaneous estimate of ocean currents!

---

## 6. Distributed Topside-Subsea Architecture & Real-Time Performance

### 6.1 3-Tier Network Topology & Tether Protocol

The system separates high-compute neural vision from safety-critical thruster control across three hardware layers:

1. **Tier 1: Topside Laptop Workstation (NVIDIA RTX 4070 GPU / Core i7)**
   - Ingests RTP H.264 video feed over Ethernet UDP port 5600/5601.
   - Runs **YOLO26 World** at $$1024 \times 1024$$ resolution ($$\sim 30\text{ ms}$$ inference).
   - Executes [`AUVVisualKalmanFilter`](file:///home/radhi/Documents/AUV_GitHub_Upload/kalman_filter.py#L31) to smooth pixel jitter and estimate target velocities.
   - Computes normalized visual servoing errors and dispatches MAVLink `MANUAL_CONTROL` packets at 30 Hz.

2. **Tier 2: Subsea Companion Computer (Raspberry Pi 4B under BlueOS 1.4.5)**
   - Encodes raw camera video via hardware H.264 pipeline into RTP stream.
   - Routes MAVLink telemetry between topside and Pixhawk.
   - Executes [`AUVDynamicsKalmanFilter`](file:///home/radhi/Documents/AUV_GitHub_Upload/kalman_filter.py#L319) at 50 Hz to estimate true vehicle velocity and current drift.

3. **Tier 3: Flight Controller (Pixhawk 2.4.8 running ArduSub 4.6)**
   - Executes internal **400 Hz EKF3** for real-time attitude estimation (quaternions).
   - Runs PID rate controllers and maps MAVLink setpoints to PWM signals across the 6x T200 thrusters.

---

### 6.2 Microsecond Execution Profiling & Algorithmic Complexity

Both filters were optimized using Python `__slots__` and pre-allocated contiguous memory buffers to eliminate memory allocation and garbage collection overhead:

| Metric | Visual Filter (`AUVVisualKalmanFilter`) | Dynamics Filter (`AUVDynamicsKalmanFilter`) |
|---|---|---|
| **State Dimension ($$n$$)** | 8 ($$\mathbf{x} \in \mathbb{R}^8$$) | 6 ($$\mathbf{x} \in \mathbb{R}^6$$) |
| **Measurement Dimension ($$m$$)** | 4 ($$\mathbf{z} \in \mathbb{R}^4$$) | 4 ($$\mathbf{z} \in \mathbb{R}^4$$) |
| **Inversion Complexity** | $$4 \times 4$$ Matrix ($$< 80$$ FLOPs) | $$4 \times 4$$ Matrix ($$< 80$$ FLOPs) |
| **Total Step FLOPs** | $$\sim 1,200$$ FLOPs | $$\sim 1,450$$ FLOPs |
| **Measured Runtime per Step** | **$$13.49\text{ \mu s}$$** | **$$20.99\text{ \mu s}$$** |
| **Maximum Throughput** | **$$74,128\text{ Hz}$$** | **$$47,641\text{ Hz}$$** |
| **CPU Utilization @ 50 Hz on Pi 4B** | Negligible (runs on laptop) | **$$< 0.5\%$$ of one core** |

---

### 6.3 Hardware-in-the-Loop (HIL) Dry Benchtop Testing Methodology

To validate the entire sensor-to-actuator pipeline before building the waterproof hull or testing in water:
1. Connect Laptop, Raspberry Pi 4B, and Pixhawk 2.4.8 on the desk via Ethernet and USB.
2. In Cockpit or MAVProxy, set `ARMING_CHECK = 0` to bypass missing water pressure sensor checks.
3. Arm in `MANUAL` mode (`arm throttle`).
4. Tilt Pixhawk by hand: Confirm artificial horizon in Cockpit tracks orientation.
5. Move target object in front of camera: Observe YOLO26 detect, `AUVVisualKalmanFilter` track, and thruster PWM outputs on `SERVO_OUTPUT_RAW` channels 1–6 dynamically respond in real time!

---

## 7. Closed-Loop Visual Servoing & Hydrodynamic Munk Moment Suppression

### 7.1 Image-Based Visual Servoing (IBVS) Interaction Matrix

The connection between 2D image-plane feature errors and 3D vehicle body velocities is governed by the **Image Interaction Matrix (Feature Jacobian)** $$\mathbf{L}_s$$:
$$\dot{\mathbf{s}} = \mathbf{L}_s \boldsymbol{\nu}$$
For a feature point at normalized coordinates $$(x_n, y_n) = \left(\frac{x - c_x}{f}, \frac{y - c_y}{f}\right)$$ with depth $$Z$$:
$$\mathbf{L}_s = \begin{bmatrix}
-\frac{1}{Z} & 0 & \frac{x_n}{Z} & x_n y_n & -(1 + x_n^2) & y_n \\
0 & -\frac{1}{Z} & \frac{y_n}{Z} & 1 + y_n^2 & -x_n y_n & -x_n
\end{bmatrix}$$

In our decoupled 4-DOF AUV control:
- Yaw rate $$r$$ is driven by horizontal error $$e_x = \frac{\hat{x} - c_x}{c_x}$$.
- Heave velocity $$w$$ is driven by vertical error $$e_y = \frac{\hat{y} - c_y}{c_y}$$.
- Surge velocity $$u$$ is driven by bounding box scale error $$e_{\text{surge}} = \frac{w_{\text{desired}} - \hat{w}}{w_{\text{desired}}}$$.

---

### 7.2 Mathematical Proof: Munk Moment Destabilization Suppression via Filtered State Feedback

A slender underwater body travelling at speed $$U$$ with angle of attack $$\alpha$$ experiences an inviscid hydrodynamic destabilizing torque known as the **Munk Moment**:
$$N_{\text{Munk}} = (M_v - M_u) u v = (Y_{\dot{v}} - X_{\dot{u}}) u v$$

For our vehicle:
$$M_v - M_u = 18.62 - 17.86 = +0.76\text{ kg} > 0$$

Because $$M_v > M_u$$, any lateral sway velocity ($$v \ne 0$$) generates a positive moment $$N_{\text{Munk}}$$ that pushes the vehicle's heading further away from its path, causing uncontrollable yaw spin without stabilization!

#### Proof of Stabilization via Kalman Filter:
Let the yaw tracking law be:
$$\tau_{\text{yaw}} = -K_p e_x - K_d \dot{e}_x$$

If raw vision measurements $$z_x$$ are used:
$$e_{x, \text{raw}} = z_x - c_x = e_x + v_k, \quad \text{where } v_k \sim \mathcal{N}(0, \sigma^2)$$

Differentiating raw noisy signals to compute derivative action results in infinite noise amplification:
$$\dot{e}_{x, \text{raw}} = \frac{(e_x(t) + v_k) - (e_x(t - \Delta t) + v_{k-1})}{\Delta t} \implies \text{Var}(\dot{e}) = \frac{2\sigma^2}{\Delta t^2}$$

At $$\Delta t = 0.033\text{ s}$$, noise variance is amplified by a factor of $$\frac{2}{(0.033)^2} \approx 1,836$$! This noise directly injects erratic PWM chatter into thrusters 1–4, inducing lateral sway vibrations ($$v$$) that trigger the Munk Moment.

**Under Kalman Filtering**:
The Kalman filter estimates the true state $$\hat{x}$$ and velocity $$\hat{v}_x$$ analytically without numerical differentiation:
$$\hat{e}_x = \hat{x} - c_x, \quad \dot{\hat{e}}_x = \hat{v}_x$$

The estimation error covariance is bounded:
$$\lim_{k \to \infty} \text{Var}(\hat{x}) = P_{11} \ll \sigma^2$$
$$\lim_{k \to \infty} \text{Var}(\hat{v}_x) = P_{55} \ll \frac{2\sigma^2}{\Delta t^2}$$

By supplying clean, noise-free state estimates to the controller, lateral sway chatter is eliminated ($$v \to 0$$), which strictly guarantees:
$$N_{\text{Munk}} = (M_v - M_u) u v \to 0$$

Thus, the Kalman filter mathematically suppresses hydrodynamic instability!

---

## 8. Numerical Walkthrough: 5-Cycle Matrix Arithmetic with Real Numbers

To illustrate the exact arithmetic executed inside the filter, we compute a 5-step numerical walk-through for a target starting at $$(300, 200)\text{ px}$$ moving right at $$50\text{ px/s}$$ with time step $$\Delta t = 0.1\text{ s}$$.

### Initial Parameters:
$$\mathbf{x}_0 = \begin{bmatrix} 300 & 50 \end{bmatrix}^T, \quad \mathbf{P}_0 = \begin{bmatrix} 10.0 & 0.0 \\ 0.0 & 10.0 \end{bmatrix}$$
$$\mathbf{A} = \begin{bmatrix} 1 & 0.1 \\ 0 & 1 \end{bmatrix}, \quad \mathbf{H} = \begin{bmatrix} 1 & 0 \end{bmatrix}, \quad q_s = 0.1, \quad R = 4.0$$
$$\mathbf{Q} = q_s \begin{bmatrix} \frac{\Delta t^3}{3} & \frac{\Delta t^2}{2} \\ \frac{\Delta t^2}{2} & \Delta t \end{bmatrix} = 0.1 \begin{bmatrix} 0.000333 & 0.005 \\ 0.005 & 0.1 \end{bmatrix} = \begin{bmatrix} 0.000033 & 0.0005 \\ 0.0005 & 0.01 \end{bmatrix}$$

---

### Cycle 1 ($$k = 1$$):
- **Raw Sensor Measurement**: $$z_1 = 307.2\text{ px}$$ (True position: 305.0 px, noise = +2.2 px).
- **1. Predict Step**:
  $$\hat{\mathbf{x}}_{1|0} = \mathbf{A} \hat{\mathbf{x}}_0 = \begin{bmatrix} 1 & 0.1 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} 300 \\ 50 \end{bmatrix} = \begin{bmatrix} 305.0 \\ 50.0 \end{bmatrix}$$
  $$\mathbf{P}_{1|0} = \mathbf{A} \mathbf{P}_0 \mathbf{A}^T + \mathbf{Q} = \begin{bmatrix} 10.200 & 1.0005 \\ 1.0005 & 10.010 \end{bmatrix}$$
- **2. Innovation**:
  $$y_1 = z_1 - \mathbf{H}\hat{\mathbf{x}}_{1|0} = 307.2 - 305.0 = +2.20\text{ px}$$
  $$S_1 = \mathbf{H} \mathbf{P}_{1|0} \mathbf{H}^T + R = 10.200 + 4.0 = 14.200$$
- **3. Optimal Gain**:
  $$\mathbf{K}_1 = \mathbf{P}_{1|0} \mathbf{H}^T S_1^{-1} = \frac{1}{14.200} \begin{bmatrix} 10.200 \\ 1.0005 \end{bmatrix} = \begin{bmatrix} 0.7183 \\ 0.0705 \end{bmatrix}$$
- **4. Correct Step**:
  $$\hat{\mathbf{x}}_{1|1} = \begin{bmatrix} 305.0 \\ 50.0 \end{bmatrix} + \begin{bmatrix} 0.7183 \\ 0.0705 \end{bmatrix} (2.20) = \begin{bmatrix} 306.58 \\ 50.15 \end{bmatrix}$$
  $$\mathbf{P}_{1|1} = (\mathbf{I} - \mathbf{K}_1 \mathbf{H})\mathbf{P}_{1|0} = \begin{bmatrix} 2.873 & 0.282 \\ 0.282 & 9.940 \end{bmatrix}$$

Notice: Measurement noise was reduced from $$+2.2\text{ px}$$ error down to $$+1.58\text{ px}$$, and position uncertainty $$\mathbf{P}_{11}$$ dropped from $$10.0$$ to $$2.87$$!

---

### Complete 5-Cycle Trajectory Progression:

| Step $$k$$ | True Pos ($$x_{\text{true}}$$) | Measured ($$z_k$$) | Predicted ($$\hat{x}_{k|k-1}$$) | Innovation ($$y_k$$) | Kalman Gain ($$K_1$$) | Filtered State ($$\hat{x}_{k|k}$$) | Error Cov ($$P_{11}$$) |
|---|---|---|---|---|---|---|---|
| **0** | 300.00 | — | — | — | — | 300.00 | 10.00 |
| **1** | 305.00 | 307.20 | 305.00 | +2.20 | 0.7183 | **306.58** | 2.87 |
| **2** | 310.00 | 308.10 | 311.59 | -3.49 | 0.4211 | **310.12** | 1.68 |
| **3** | 315.00 | 317.90 | 315.13 | +2.77 | 0.3015 | **315.96** | 1.21 |
| **4** (Occluded) | 320.00 | **None** | **320.97** | 0.00 | 0.0000 | **320.97** | 1.58 |
| **5** | 325.00 | 324.20 | 326.00 | -1.80 | 0.2830 | **325.49** | 1.13 |

At step 4, complete occlusion occurred (camera detected nothing). The filter dead-reckoned to $$320.97\text{ px}$$ (true was $$320.00\text{ px}$$, error just $$0.97\text{ px}$$!), allowing uninterrupted visual tracking!

---

## 9. Complete Parameter Taxonomy & Variable Hierarchy Dictionary

| Category | Variable / Symbol | Formal Mathematical Definition | Physical Value / Unit |
|---|---|---|---|
| **Visual Filter** | $$\mathbf{x}_{\text{vis}} \in \mathbb{R}^8$$ | Full visual state vector ($$[x, y, w, h, v_x, v_y, v_w, v_h]^T$$) | Pixels, px/s |
| | $$\mathbf{z}_k \in \mathbb{R}^4$$ | Visual measurement vector ($$[x_m, y_m, w_m, h_m]^T$$) | Pixels |
| | $$\mathbf{A}(\Delta t) \in \mathbb{R}^{8\times 8}$$ | Discrete state transition matrix | Dimensionless |
| | $$\mathbf{Q}_{\text{vis}} \in \mathbb{R}^{8\times 8}$$ | Process noise covariance matrix | CWNA model ($$q_s = 0.05$$) |
| | $$\mathbf{R}_{\text{vis}} \in \mathbb{R}^{4\times 4}$$ | Adaptive measurement noise covariance | $$\mathbf{R}_0 / \max(\text{conf}, 0.15)^2$$ |
| | $$\mathcal{A}, \dot{\mathcal{A}}$$ | Projected bounding box area and expansion rate | $$\text{px}^2, \text{px}^2/\text{s}$$ |
| **Dynamics Filter** | $$\mathbf{x}_{\text{dyn}} \in \mathbb{R}^6$$ | Hydrodynamic state vector ($$[u, v, w, r, d_u, d_v]^T$$) | m/s, rad/s, N |
| | $$M_u, M_v, M_w, M_r$$ | Generalized vehicle inertia entries ($$\mathbf{M}_{RB} + \mathbf{M}_A$$) | $$17.86, 18.62, 30.18\text{ kg}, 0.25\text{ kg}\cdot\text{m}^2$$ |
| | $$\mathbf{D}_{\text{lin}}$$ | Linear laminar damping vector ($$[X_u, Y_v, Z_w, N_r]^T$$) | $$[13.7, 0.0, 33.8, 0.0]^T\text{ Ns/m}$$ |
| | $$\mathbf{D}_{\text{quad}}$$ | Quadratic turbulent form drag vector ($$[X_{u|u|}, Y_{v|v|}, Z_{w|w|}, N_{r|r|}]^T$$) | $$[141.0, 217.0, 190.0, 1.5]^T\text{ Ns}^2/\text{m}^2$$ |
| | $$d_u, d_v$$ | Estimated ocean current disturbance forces | Newtons (N) |
| | $$\mathbf{F}_c \in \mathbb{R}^{6\times 6}$$ | Analytical continuous dynamics Jacobian matrix | Evaluated online at current $$\hat{\boldsymbol{\nu}}$$ |
| | $$\boldsymbol{\Phi} \in \mathbb{R}^{6\times 6}$$ | Discrete state transition matrix | $$\mathbf{I}_6 + \mathbf{F}_c \Delta t$$ |
| **Control Laws** | $$e_x, e_y, e_{\text{surge}}$$ | Normalized visual servoing errors | $$[-1.0, +1.0]$$ dimensionless |
| | $$\tau_{\text{yaw}}, \tau_{\text{heave}}, \tau_{\text{surge}}$$ | Commanded thruster actuation efforts | $$[-400, +400]$$ PWM units |

---

## 10. Comprehensive Master Bibliography

1. **Fossen, T. I.** (2021). *Handbook of Marine Craft Hydrodynamics and Motion Control* (2nd ed.). John Wiley & Sons. — Definitive reference for marine craft 6-DOF kinematics, hydrodynamic added mass, damping modeling, and non-linear observer design.
2. **Kim, Y. V.** (Ed.). (2023). *Kalman Filter - Engineering Applications*. IntechOpen. ISBN: 978-1-80356-575-0, DOI: 10.5772/intechopen.100722. — Comprehensive treatment of discrete Kalman filters in aerospace, robotics, and navigation.
3. **Khalid, A., Sarwat, A., & Riggs, H.** (Eds.). (2024). *Applications and Optimizations of Kalman Filter and Their Variants*. IntechOpen. ISBN: 978-0-85466-565-5. — High-speed implementations, adaptive noise tuning, and computational optimizations.
4. **Särkkä, S., & Svensson, L.** (2023). *Bayesian Filtering and Smoothing* (2nd ed.). Cambridge University Press. DOI: 10.1017/9781108910002. — Rigorous mathematical derivations of optimal Bayesian estimators, continuous-discrete EKF, and Riccati differential solvers.
5. **Kalman, R. E.** (1960). "A New Approach to Linear Filtering and Prediction Problems". *Journal of Basic Engineering*, 82(1), 35–45. — The original landmark paper.
6. **Chaumette, F., & Hutchinson, S.** (2006). "Visual Servo Control Part I: Basic Approaches". *IEEE Robotics & Automation Magazine*, 13(4), 82–90. — Foundational theory for Image-Based Visual Servoing (IBVS).
7. **Ultralytics**. (2026). *YOLO26 & YOLO-World Open-Vocabulary Architecture Documentation*. https://docs.ultralytics.com/
