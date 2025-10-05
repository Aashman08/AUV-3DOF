# AUV 6-DOF Physics Correction Plan
## Modular, Risk-Managed Implementation Strategy

**Date:** October 5, 2025  
**Prerequisites:** Complete `Physics-Audit.md` review and approval  
**Objective:** Correct fundamental physics violations while maintaining system stability

---

# OVERVIEW & PHILOSOPHY

## Guiding Principles

1. **Atomic changes:** Each patch addresses ONE root cause
2. **Test before advancing:** Validation tests for each step
3. **Backward compatibility:** Maintain existing interfaces where possible
4. **Configuration-driven:** All new parameters in `config.yaml`
5. **Documentation inline:** Update docstrings with equations
6. **Commit discipline:** One conceptual change per commit

## Risk Management Strategy

**Phases:**
- **Phase 1:** Kinematic corrections (low risk, high impact)
- **Phase 2:** Mass/inertia matrix completeness (medium risk)
- **Phase 3:** Force coupling (high risk, test extensively)
- **Phase 4:** Control improvements (medium risk)
- **Phase 5:** Numerical upgrades (low risk)

**Testing at each phase:**
- Unit tests for new functions
- Integration test (full simulation run)
- Energy conservation check
- Compare before/after plots
- Document behavior changes

---

# PATCH SEQUENCE

---

## PATCH 1: Frame & Units Normalization

**Objective:** Fix Euler angle kinematics and ensure NED consistency throughout

**Root Cause Addressed:** RC#1 (§2.1.3 of audit)

### 1.1 Mathematical Formulation

**Correct Euler angle kinematics (Fossen Eq. 2.29):**

```
⎡φ̇⎤   ⎡1   sin(φ)tan(θ)   cos(φ)tan(θ)⎤ ⎡p⎤
⎢θ̇⎥ = ⎢0   cos(φ)        -sin(φ)      ⎥·⎢q⎥  = T(η) · ν₂
⎣ψ̇⎦   ⎣0   sin(φ)sec(θ)   cos(φ)sec(θ)⎦ ⎣r⎦
```

**Singularity condition:** θ = ±π/2 (cos(θ) → 0, matrix T becomes singular)

### 1.2 Implementation Changes

**File:** `src/physics/vehicle_dynamics.py`

**Add transformation function:**
```python
def _body_rates_to_euler_rates(self, euler_angles: np.ndarray, 
                                body_rates: np.ndarray) -> np.ndarray:
    """
    Transform body angular rates [p,q,r] to Euler angle rates [φ̇,θ̇,ψ̇].
    
    Implements the transformation matrix T(η) from Fossen (2011) Eq. 2.29:
    η̇₂ = T(η₂) · ν₂
    
    Args:
        euler_angles: [φ, θ, ψ] in radians
        body_rates: [p, q, r] in rad/s
        
    Returns:
        Euler rates [φ̇, θ̇, ψ̇] in rad/s
        
    Raises:
        RuntimeWarning: If |θ| > 80° (approaching gimbal lock)
    """
    phi, theta, psi = euler_angles
    p, q, r = body_rates
    
    # Check for gimbal lock proximity
    if abs(theta) > np.deg2rad(80):
        logger.warning(f"Pitch angle {np.rad2deg(theta):.1f}° approaching gimbal lock (±90°)")
    
    # Compute transformation matrix T
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    tan_theta = np.tan(theta)
    
    # Guard against singularity
    if abs(cos_theta) < 0.01:
        # At gimbal lock, use approximation or fall back
        logger.error(f"Gimbal lock at theta={np.rad2deg(theta):.1f}°")
        # Degenerate case: hold current orientation, log warning
        return np.array([p, 0.0, 0.0])  # Only roll rate survives
    
    sec_theta = 1.0 / cos_theta
    
    T = np.array([
        [1.0,  sin_phi * tan_theta,  cos_phi * tan_theta],
        [0.0,  cos_phi,              -sin_phi],
        [0.0,  sin_phi * sec_theta,  cos_phi * sec_theta]
    ])
    
    euler_rates = T @ body_rates
    return euler_rates
```

**Modify integration in `integrate_dynamics()` (lines 259-280):**
```python
# === ANGULAR KINEMATICS (CORRECTED) ===
# Transform body rates to Euler rates using proper transformation
euler_rates = self._body_rates_to_euler_rates(
    np.array([roll, pitch, yaw]),
    np.array([p_new, q_new, r_new])
)
phi_dot, theta_dot, psi_dot = euler_rates

# Integrate Euler angles
roll_new = roll + phi_dot * dt
pitch_new = pitch + theta_dot * dt
yaw_new = yaw + psi_dot * dt
```

**Remove incorrect lines:**
```python
# DELETE these lines:
# roll_new = roll + p * dt      # WRONG
# pitch_new = pitch + q * dt    # WRONG
# yaw_new = yaw + r * dt        # WRONG
```

### 1.3 Quaternion Alternative (Optional Enhancement)

**File:** `src/data_types/types.py`

**Add quaternion state representation:**
```python
@dataclass
class VehicleStateQuaternion:
    """Alternative vehicle state using quaternions for attitude."""
    timestamp: float = 0.0
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))  # [q0, qx, qy, qz]
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    angular_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    def normalize_quaternion(self):
        """Ensure quaternion has unit norm."""
        norm = np.linalg.norm(self.quaternion)
        if norm > 1e-6:
            self.quaternion /= norm
        else:
            self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Reset to identity
```

**Add quaternion kinematics function:**
```python
def quaternion_derivative(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    """
    Compute quaternion time derivative from angular velocity.
    
    q̇ = 0.5 * Ω(ω) * q
    
    where Ω(ω) is the skew-symmetric matrix:
    Ω = [  0   -p   -q   -r ]
        [  p    0    r   -q ]
        [  q   -r    0    p ]
        [  r    q   -p    0 ]
    
    Args:
        q: Quaternion [q0, qx, qy, qz]
        omega: Angular velocity [p, q, r] in rad/s
        
    Returns:
        Quaternion derivative [q̇0, q̇x, q̇y, q̇z]
    """
    q0, qx, qy, qz = q
    p, q_ang, r = omega  # Note: 'q' conflicts with quaternion element
    
    q_dot = 0.5 * np.array([
        -p*qx - q_ang*qy - r*qz,
         p*q0 + r*qy - q_ang*qz,
         q_ang*q0 - r*qx + p*qz,
         r*q0 + q_ang*qx - p*qy
    ])
    
    return q_dot
```

**Configuration option:**
```yaml
# In config.yaml
simulation:
  attitude_representation: "euler"  # "euler" or "quaternion"
  euler_gimbal_lock_limit: 80.0    # [deg] warn if |pitch| exceeds
```

### 1.4 Validation Tests

**Test 1.1: Identity transformation**
```python
def test_euler_kinematics_identity():
    """At zero Euler angles, T = I (identity)."""
    dynamics = VehicleDynamics(config)
    euler = np.array([0.0, 0.0, 0.0])
    body_rates = np.array([0.1, 0.2, 0.3])
    
    euler_rates = dynamics._body_rates_to_euler_rates(euler, body_rates)
    
    np.testing.assert_array_almost_equal(euler_rates, body_rates, decimal=10)
```

**Test 1.2: Roll-only motion**
```python
def test_euler_kinematics_roll_only():
    """Pure roll rate: φ̇ = p, θ̇ = 0, ψ̇ = 0."""
    dynamics = VehicleDynamics(config)
    euler = np.array([0.5, 0.0, 0.0])  # Some roll, zero pitch/yaw
    body_rates = np.array([1.0, 0.0, 0.0])  # Pure roll rate
    
    euler_rates = dynamics._body_rates_to_euler_rates(euler, body_rates)
    
    assert abs(euler_rates[0] - 1.0) < 1e-10  # φ̇ ≈ p
    assert abs(euler_rates[1]) < 1e-10        # θ̇ ≈ 0
    assert abs(euler_rates[2]) < 1e-10        # ψ̇ ≈ 0
```

**Test 1.3: Pitch coupling**
```python
def test_euler_kinematics_pitch_coupling():
    """At non-zero pitch, roll and yaw rates couple."""
    dynamics = VehicleDynamics(config)
    euler = np.array([0.0, np.deg2rad(30), 0.0])  # 30° pitch
    body_rates = np.array([0.0, 0.0, 1.0])  # Pure yaw rate
    
    euler_rates = dynamics._body_rates_to_euler_rates(euler, body_rates)
    
    # At θ=30°, yaw rate r couples into roll rate: φ̇ = r·cos(φ)·tan(θ)
    expected_phi_dot = 1.0 * np.cos(0.0) * np.tan(np.deg2rad(30))
    expected_psi_dot = 1.0 / np.cos(np.deg2rad(30))
    
    assert abs(euler_rates[0] - expected_phi_dot) < 1e-6
    assert abs(euler_rates[2] - expected_psi_dot) < 1e-6
```

**Test 1.4: Gimbal lock detection**
```python
def test_euler_kinematics_gimbal_lock_warning():
    """Should warn when |pitch| > 80°."""
    dynamics = VehicleDynamics(config)
    euler = np.array([0.0, np.deg2rad(85), 0.0])
    body_rates = np.array([0.1, 0.1, 0.1])
    
    with pytest.warns(RuntimeWarning, match="gimbal lock"):
        euler_rates = dynamics._body_rates_to_euler_rates(euler, body_rates)
```

### 1.5 Expected Behavior Changes

**Before:**
- Orientation drifts during combined maneuvers
- Heading error accumulates in spiral turns
- Pitch/roll coupling absent

**After:**
- Correct orientation evolution
- Accurate heading tracking
- Realistic gimbal lock warnings

**Performance impact:** Negligible (3×3 matrix multiply added, ~10 flops)

**Risk:** Low (kinematics are well-established)

### 1.6 Documentation Updates

**README.md update:**
```markdown
## Physics Model

### Kinematics
The simulation uses **Euler angles** (roll φ, pitch θ, yaw ψ) with proper kinematic transformation:
```
η̇₂ = T(η₂) · ν₂
```
where T is the Euler rate transformation matrix (Fossen 2011, Eq. 2.29).

**Gimbal lock:** Singularity occurs at pitch = ±90°. The system logs warnings at |θ| > 80°.

**Alternative:** Quaternion representation available via config option (eliminates singularities).
```

**Commit message:**
```
fix(physics): correct Euler angle kinematics with proper transformation matrix

- Add T(η) transformation from body rates to Euler rates
- Implement gimbal lock detection and warning
- Add quaternion option for robust attitude (config-selectable)
- Add unit tests for kinematic transformation
- Update documentation with frame conventions

Fixes: Incorrect direct integration φ̇=p, θ̇=q, ψ̇=r
Refs: Physics-Audit.md §2.1.3 (ROOT CAUSE #1)
```

---

## PATCH 2: Rigid-Body Mass & Inertia Matrix

**Objective:** Complete the mass/inertia parameters and structure M_RB, M_A

**Root Cause Addressed:** RC#4 (§2.2.2 of audit)

### 2.1 Configuration Additions

**File:** `config/config.yaml`

**Add missing hydrodynamic parameters:**
```yaml
hydrodynamics:
  # Existing parameters...
  d1_surge: 0.0
  d2_surge: 35.0
  added_mass_surge: 9.0
  added_mass_sway: 90.0    # Already present, now used
  added_mass_heave: 90.0
  
  # NEW: Added inertia for rotational DOFs
  added_inertia_roll: 0.23      # [kg·m²] ~10% of I_xx (axial, small)
  added_inertia_pitch: 52.7     # [kg·m²] ~30% of I_yy (lateral, large)
  added_inertia_yaw: 52.7       # [kg·m²] ~30% of I_zz (lateral, large)
  
  # NEW: Sway and heave damping (quadratic cross-flow)
  d2_sway: 120.0                # [N·s²/m²] Higher than surge (bluff body)
  d2_heave: 120.0               # [N·s²/m²] Same as sway (symmetric)
  
  # NEW: Quadratic angular damping (for high-rate maneuvers)
  c_roll_rate_quadratic: 0.5    # [N·m·s²] Small for torpedo
  c_pitch_rate_quadratic: 10.0  # [N·m·s²]
  c_yaw_rate_quadratic: 10.0    # [N·m·s²]
  
  # Existing rotational damping...
  c_roll_rate: 2.0
  c_pitch_rate: 35.0
  c_yaw_rate: 35.0
```

**Rationale for values:**

**Added inertia (Fossen Table 6.1, torpedo approximation):**
- Roll (axial): ~10% of I_xx (small, streamlined)
- Pitch/Yaw (lateral): ~30% of I_yy, I_zz (significant, cross-flow)

**Cross-flow damping:**
- Sway/heave experience bluff-body drag (not streamlined)
- Factor ~3× higher than surge quadratic drag: 120 vs 35 N·s²/m²

**Quadratic angular damping:**
- Becomes significant at |ω| > 10°/s = 0.175 rad/s
- Moment contribution: M_quadratic = -c_quad · ω · |ω|
- At ω = 20°/s: M_quad_pitch = -10 · 0.349² = -1.22 N·m (non-negligible)

### 2.2 Code Changes

**File:** `src/physics/vehicle_dynamics.py`

**Update initialization (lines 89-131):**
```python
def __init__(self, config: Dict[str, Any]):
    # ... (existing vehicle parameters) ...
    
    # Extract hydrodynamic coefficients (UPDATED)
    hydro_cfg = config['hydrodynamics']
    
    # Added mass for linear motion
    self.added_mass_surge = hydro_cfg['added_mass_surge']  # [kg]
    self.added_mass_sway = hydro_cfg['added_mass_sway']    # [kg] NOW USED
    self.added_mass_heave = hydro_cfg['added_mass_heave']  # [kg]
    
    # Added inertia for rotational motion (NEW)
    self.added_inertia_roll = hydro_cfg['added_inertia_roll']    # [kg·m²]
    self.added_inertia_pitch = hydro_cfg['added_inertia_pitch']  # [kg·m²]
    self.added_inertia_yaw = hydro_cfg['added_inertia_yaw']      # [kg·m²]
    
    # Linear damping
    self.d1_surge = hydro_cfg['d1_surge']
    
    # Quadratic damping (EXPANDED)
    self.d2_surge = hydro_cfg['d2_surge']
    self.d2_sway = hydro_cfg['d2_sway']      # NEW
    self.d2_heave = hydro_cfg['d2_heave']    # NEW
    
    # Angular damping - linear
    self.c_roll_rate = hydro_cfg['c_roll_rate']
    self.c_pitch_rate = hydro_cfg['c_pitch_rate']
    self.c_yaw_rate = hydro_cfg['c_yaw_rate']
    
    # Angular damping - quadratic (NEW)
    self.c_roll_rate_quadratic = hydro_cfg['c_roll_rate_quadratic']
    self.c_pitch_rate_quadratic = hydro_cfg['c_pitch_rate_quadratic']
    self.c_yaw_rate_quadratic = hydro_cfg['c_yaw_rate_quadratic']
    
    # ... (environmental parameters) ...
    
    # Total system mass matrix: M = M_RB + M_A (UPDATED)
    self.total_mass_surge = self.mass + self.added_mass_surge
    self.total_mass_sway = self.mass + self.added_mass_sway      # NOW USED
    self.total_mass_heave = self.mass + self.added_mass_heave
    
    # Total inertia matrix (CORRECTED - no longer defaults to 0)
    self.total_inertia_roll = self.I_xx + self.added_inertia_roll
    self.total_inertia_pitch = self.I_yy + self.added_inertia_pitch
    self.total_inertia_yaw = self.I_zz + self.added_inertia_yaw
    
    # Validate mass matrix positive-definiteness (NEW)
    self._validate_mass_matrix()
    
    logger.info(f"Vehicle dynamics initialized:")
    logger.info(f"  Total mass: surge={self.total_mass_surge:.1f}kg, "
                f"sway={self.total_mass_sway:.1f}kg, heave={self.total_mass_heave:.1f}kg")
    logger.info(f"  Total inertia: roll={self.total_inertia_roll:.2f}kg·m², "
                f"pitch={self.total_inertia_pitch:.1f}kg·m², yaw={self.total_inertia_yaw:.1f}kg·m²")
```

**Add validation method:**
```python
def _validate_mass_matrix(self):
    """Validate that mass/inertia parameters are physically realizable."""
    # Check positive-definiteness (all masses/inertias > 0)
    mass_vector = np.array([
        self.total_mass_surge,
        self.total_mass_sway,
        self.total_mass_heave,
        self.total_inertia_roll,
        self.total_inertia_pitch,
        self.total_inertia_yaw
    ])
    
    if np.any(mass_vector <= 0):
        raise ValueError(f"Mass matrix has non-positive elements: {mass_vector}")
    
    # Check reasonable ratios (added mass should be < 200% of rigid-body)
    if self.added_mass_surge > 2.0 * self.mass:
        logger.warning(f"Added mass surge ({self.added_mass_surge}kg) > 200% of vehicle mass")
    
    # Check inertia ordering (typically I_yy ≈ I_zz >> I_xx for torpedo)
    if self.I_xx > 0.2 * self.I_yy:
        logger.warning(f"Roll inertia unusually large: I_xx={self.I_xx:.1f}, I_yy={self.I_yy:.1f}")
    
    logger.debug("Mass matrix validation passed")
```

**Update damping forces (lines 175-206):**
```python
# === ROLL DYNAMICS (UPDATED) ===
# Linear damping
roll_damping_linear = -self.c_roll_rate * p

# Quadratic damping (NEW)
roll_damping_quadratic = -self.c_roll_rate_quadratic * p * abs(p)

# Roll moment from differential fin deflection
roll_lift_moment = dynamic_pressure * self.fin_area * self.CL_delta * self.tail_arm * 0.25 * (
    (fin_angles[0] + fin_angles[3]) - (fin_angles[1] + fin_angles[2])
)

forces.roll_moment = roll_damping_linear + roll_damping_quadratic + roll_lift_moment

# === PITCH DYNAMICS (UPDATED) ===
pitch_damping_linear = -self.c_pitch_rate * q
pitch_damping_quadratic = -self.c_pitch_rate_quadratic * q * abs(q)  # NEW

pitch_lift_moment = dynamic_pressure * self.fin_area * self.CL_delta * self.tail_arm * 0.5 * (
    (fin_angles[2] + fin_angles[3]) - (fin_angles[0] + fin_angles[1])
)

forces.pitch_moment = pitch_damping_linear + pitch_damping_quadratic + pitch_lift_moment

# === YAW DYNAMICS (UPDATED) ===
yaw_damping_linear = -self.c_yaw_rate * r
yaw_damping_quadratic = -self.c_yaw_rate_quadratic * r * abs(r)  # NEW

yaw_lift_moment = dynamic_pressure * self.fin_area * self.CL_delta * self.tail_arm * 0.5 * (
    (fin_angles[1] + fin_angles[2]) - (fin_angles[0] + fin_angles[3])
)

forces.yaw_moment = yaw_damping_linear + yaw_damping_quadratic + yaw_lift_moment
```

### 2.3 Validation Tests

**Test 2.1: Mass matrix validation**
```python
def test_mass_matrix_positive_definite():
    """All mass/inertia terms must be positive."""
    config_valid = load_config("config/config.yaml")
    dynamics = VehicleDynamics(config_valid)
    
    assert dynamics.total_mass_surge > 0
    assert dynamics.total_mass_sway > 0
    assert dynamics.total_mass_heave > 0
    assert dynamics.total_inertia_roll > 0
    assert dynamics.total_inertia_pitch > 0
    assert dynamics.total_inertia_yaw > 0

def test_mass_matrix_invalid_raises():
    """Invalid config (negative mass) should raise ValueError."""
    config_invalid = load_config("config/config.yaml")
    config_invalid['vehicle']['mass'] = -10.0  # Invalid
    
    with pytest.raises(ValueError, match="non-positive"):
        dynamics = VehicleDynamics(config_invalid)
```

**Test 2.2: Added inertia effect**
```python
def test_added_inertia_slows_rotation():
    """Added inertia should reduce angular acceleration."""
    config = load_config("config/config.yaml")
    
    # Case 1: No added inertia
    config['hydrodynamics']['added_inertia_pitch'] = 0.0
    dyn_no_added = VehicleDynamics(config)
    
    # Case 2: With added inertia
    config['hydrodynamics']['added_inertia_pitch'] = 52.7
    dyn_with_added = VehicleDynamics(config)
    
    # Same moment should produce less acceleration with added inertia
    assert dyn_with_added.total_inertia_pitch > dyn_no_added.total_inertia_pitch
    
    # q̇ = M / I_total
    M_test = 10.0  # N·m
    q_dot_no_added = M_test / dyn_no_added.total_inertia_pitch
    q_dot_with_added = M_test / dyn_with_added.total_inertia_pitch
    
    assert q_dot_with_added < q_dot_no_added
```

**Test 2.3: Quadratic damping at high rates**
```python
def test_quadratic_damping_dominates_at_high_rate():
    """At high angular rates, quadratic damping should dominate."""
    dynamics = VehicleDynamics(load_config("config/config.yaml"))
    
    q_low = np.deg2rad(5)   # 5°/s
    q_high = np.deg2rad(30)  # 30°/s
    
    # Linear damping
    M_linear_low = dynamics.c_pitch_rate * q_low
    M_linear_high = dynamics.c_pitch_rate * q_high
    
    # Quadratic damping
    M_quad_low = dynamics.c_pitch_rate_quadratic * q_low * abs(q_low)
    M_quad_high = dynamics.c_pitch_rate_quadratic * q_high * abs(q_high)
    
    # At low rate: linear > quadratic
    assert M_linear_low > M_quad_low
    
    # At high rate: quadratic >> linear
    assert M_quad_high > M_linear_high
```

### 2.4 Expected Behavior Changes

**Before:**
- Rotational dynamics unrealistically fast (no added inertia)
- Angular damping under-predicted at high rates

**After:**
- More realistic angular accelerations (slower spin-up/down)
- Better damping at aggressive maneuvers (>15°/s)

**Performance impact:** Negligible (same computation, different coefficients)

**Risk:** Low (parameters are scalar additions)

### 2.5 Commit Message

```
feat(physics): complete mass/inertia matrix with added mass components

- Add added_inertia_roll/pitch/yaw to config (no longer defaults to 0)
- Add sway/heave quadratic damping coefficients
- Add quadratic angular damping for high-rate maneuvers
- Implement mass matrix validation (positive-definiteness check)
- Update initialization to use all 6 DOF masses/inertias

Fixes: Missing added mass effects in rotation, incomplete damping
Refs: Physics-Audit.md §2.2.2, §2.3.3 (ROOT CAUSE #4, #5, #6)
```

---

## PATCH 3: Sway Dynamics & Lateral Forces

**Objective:** Implement missing sway equation v̇ and lateral damping

**Root Cause Addressed:** RC#2 (§2.1.4 of audit)

### 3.1 Mathematical Formulation

**Sway force equation:**

```
(m + Y_v̇) · v̇ = F_sway_damping + F_sway_coriolis + F_sway_current + F_sway_fins

where:
  F_sway_damping = -D_v · v · |v|        (quadratic cross-flow drag)
  F_sway_coriolis = -m · r · u           (yaw-surge coupling, from C_RB)
  F_sway_coriolis_A = -(X_u̇ - Z_ẇ) · r · u  (added mass effect)
  F_sway_current = (drag on relative velocity)
  F_sway_fins = (roll/yaw fin deflection coupling)
```

**Note:** Full Coriolis implementation in Patch 4; for now, add basic structure.

### 3.2 Implementation

**File:** `src/physics/vehicle_dynamics.py`

**Expand `HydrodynamicForces` dataclass (lines 49-57):**
```python
@dataclass
class HydrodynamicForces:
    """Container for hydrodynamic forces and moments acting on the vehicle."""
    surge_force: float = 0.0      # [N] Force in x-direction (body frame)
    sway_force: float = 0.0       # [N] Force in y-direction (body frame) - NOW ACTIVE
    heave_force: float = 0.0      # [N] Force in z-direction (body frame)
    roll_moment: float = 0.0      # [N·m] Moment about x-axis (body frame)
    pitch_moment: float = 0.0     # [N·m] Moment about y-axis (body frame)
    yaw_moment: float = 0.0       # [N·m] Moment about z-axis (body frame)
```

**Add sway damping in `compute_hydrodynamic_forces()` (after line 206):**
```python
# === SWAY DYNAMICS (NEW) ===
# Quadratic damping from cross-flow drag
sway_damping = -self.d2_sway * v * abs(v)

# Coriolis coupling (simplified for now, full C(ν) in Patch 4)
# From rigid-body term: -m · r · u
sway_coriolis_RB = -self.mass * r * u

# TODO Patch 4: Add full Coriolis matrix including added mass effects

forces.sway_force = sway_damping + sway_coriolis_RB
```

**Update integration in `integrate_dynamics()` (lines 254-307):**
```python
# === SURGE EQUATION ===
u_dot = forces.surge_force / self.total_mass_surge
u_new = u + u_dot * dt

# === SWAY EQUATION (NEW - PREVIOUSLY MISSING) ===
v_dot = forces.sway_force / self.total_mass_sway
v_new = v + v_dot * dt

# === HEAVE EQUATION (to be refined in Patch 5) ===
# ...existing heave code...

# === UPDATE STATE ===
new_state.velocity = np.array([u_new, v_new, w_new])  # v_new now updated!
```

**Update position integration (lines 297-310):**
```python
# === HORIZONTAL POSITION (CORRECTED) ===
# Transform body velocity to global frame (proper full transformation)
R_bn = euler_to_rotation_matrix(roll, pitch, yaw)
v_body = np.array([u, v, w])
v_ned = R_bn @ v_body

x_dot, y_dot, z_dot = v_ned

x_new = x + x_dot * dt
y_new = y + y_dot * dt
z_new = z + z_dot * dt

# DELETE old simplified code:
# x_dot = u * np.cos(yaw) * np.cos(pitch)
# y_dot = u * np.sin(yaw) * np.cos(pitch)
```

**Import rotation matrix function at top:**
```python
from data_types.types import VehicleState, EnvironmentState, euler_to_rotation_matrix
```

### 3.3 Validation Tests

**Test 3.1: Sway velocity evolves**
```python
def test_sway_velocity_updates():
    """Sway velocity v should change when lateral force applied."""
    config = load_config("config/config.yaml")
    dynamics = VehicleDynamics(config)
    
    state = VehicleState()
    state.velocity = np.array([1.0, 0.0, 0.0])  # Surge only
    state.angular_velocity = np.array([0.0, 0.0, 0.5])  # Yaw rate 0.5 rad/s
    
    # Compute forces (should include sway Coriolis: -m·r·u)
    forces = dynamics.compute_hydrodynamic_forces(
        state, fin_angles=np.zeros(4), thrust=0.0, environment=EnvironmentState()
    )
    
    # Sway force should be non-zero due to Coriolis
    expected_sway_coriolis = -180 * 0.5 * 1.0  # -90 N
    assert abs(forces.sway_force - expected_sway_coriolis) < 1.0  # Within 1N
    
    # Integrate one step
    state_new = dynamics.integrate_dynamics(state, forces, dt=0.01)
    
    # Sway velocity should have changed
    assert state_new.velocity[1] != 0.0  # v is no longer stuck at 0
    
    # Expected: v̇ = F_sway / m_sway = -90 / (180+90) = -0.333 m/s²
    # After dt=0.01s: v ≈ -0.00333 m/s
    assert abs(state_new.velocity[1] - (-0.00333)) < 0.0005
```

**Test 3.2: Sway damping dissipates lateral velocity**
```python
def test_sway_damping():
    """Sway damping should reduce lateral velocity over time."""
    config = load_config("config/config.yaml")
    dynamics = VehicleDynamics(config)
    
    state = VehicleState()
    state.velocity = np.array([0.0, 1.0, 0.0])  # Pure sway, no surge
    
    env = EnvironmentState()
    
    # Run simulation for 10 seconds
    dt = 0.01
    for _ in range(1000):
        forces = dynamics.compute_hydrodynamic_forces(state, np.zeros(4), 0.0, env)
        state = dynamics.integrate_dynamics(state, forces, dt)
    
    # Sway velocity should decay to near zero (damping dominant)
    assert abs(state.velocity[1]) < 0.1  # Decayed significantly
    assert state.velocity[1] < 1.0        # Decreased from initial 1.0 m/s
```

**Test 3.3: Turn with sideslip (Coriolis)**
```python
def test_turn_induces_sway():
    """During yaw, Coriolis force should induce sway velocity."""
    config = load_config("config/config.yaml")
    dynamics = VehicleDynamics(config)
    
    state = VehicleState()
    state.velocity = np.array([2.0, 0.0, 0.0])  # 2 m/s surge
    state.angular_velocity = np.array([0.0, 0.0, 0.2])  # 0.2 rad/s yaw (turning)
    
    env = EnvironmentState()
    dt = 0.01
    
    # Run for 5 seconds
    for _ in range(500):
        forces = dynamics.compute_hydrodynamic_forces(state, np.zeros(4), 0.0, env)
        state = dynamics.integrate_dynamics(state, forces, dt)
    
    # Should develop non-zero sway velocity due to turn
    assert abs(state.velocity[1]) > 0.01  # Some lateral velocity developed
```

### 3.4 Expected Behavior Changes

**Before:**
- Sway velocity permanently zero (v stuck at initial value)
- No lateral velocity during turns (unrealistic "rail" motion)
- No response to lateral disturbances

**After:**
- Sway velocity evolves naturally
- Turns exhibit crab angle / sideslip
- Vehicle responds to lateral forces

**Performance impact:** Minimal (one additional force term, one additional integration)

**Risk:** Medium (introduces new DOF interaction)

### 3.5 Commit Message

```
feat(physics): implement sway dynamics (lateral motion)

- Add sway force equation with quadratic damping
- Add rigid-body Coriolis coupling (-m·r·u for sway)
- Update state integration to evolve sway velocity v
- Fix position integration using full rotation matrix transformation
- Add tests for sway velocity evolution and turn-induced sideslip

Previously: v was frozen at 0, vehicle could not move laterally
Now: Full lateral dynamics active, realistic turn behavior

Refs: Physics-Audit.md §2.1.4 (ROOT CAUSE #2)
```

---

## PATCH 4: Coriolis & Centripetal Forces (Full C(ν)ν)

**Objective:** Implement complete rigid-body and added mass Coriolis matrices

**Root Cause Addressed:** RC#3 (§2.2.3 of audit)

**Complexity:** HIGH — This is the most mathematically intensive patch

### 4.1 Mathematical Formulation

**Complete Coriolis/centripetal matrix (Fossen Eq. 6.43):**

For a vehicle with CG at origin (x_g = y_g = z_g = 0):

```
C(ν) = C_RB(ν) + C_A(ν)

where ν = [u, v, w, p, q, r]ᵀ
```

**Rigid-body Coriolis C_RB(ν):**

```
C_RB(ν) = [ 0₃ₓ₃       -m·S(ν₁)    ]
          [ -m·S(ν₁)   -S(I_o·ν₂)  ]

where:
  ν₁ = [u, v, w]ᵀ (linear velocities)
  ν₂ = [p, q, r]ᵀ (angular velocities)
  S(·) = skew-symmetric matrix operator
  I_o = diag([I_xx, I_yy, I_zz]) (inertia tensor, diagonal for torpedo)

S([a, b, c]ᵀ) = [  0  -c   b ]
                [  c   0  -a ]
                [ -b   a   0 ]
```

**Expanded C_RB(ν) for forces:**
```
F_x_coriolis = m·(v·r - w·q)
F_y_coriolis = m·(w·p - u·r)
F_z_coriolis = m·(u·q - v·p)
```

**Expanded C_RB(ν) for moments:**
```
L_coriolis = (I_zz - I_yy)·q·r
M_coriolis = (I_xx - I_zz)·p·r
N_coriolis = (I_yy - I_xx)·p·q
```

**Added mass Coriolis C_A(ν) (Fossen Eq. 6.50):**

For diagonal M_A = diag([X_u̇, Y_v̇, Z_ẇ, K_ṗ, M_q̇, N_ṙ]):

```
C_A(ν) = [ 0₃ₓ₃              -S(M_A_linear·ν₁) ]
         [ -S(M_A_linear·ν₁)  -S(M_A_angular·ν₂) ]

where:
  M_A_linear = diag([X_u̇, Y_v̇, Z_ẇ])
  M_A_angular = diag([K_ṗ, M_q̇, N_ṙ])
```

**Expanded C_A(ν) for forces:**
```
F_x_added = 0  (diagonal M_A, no surge coupling)
F_y_added = (Z_ẇ - X_u̇)·w·r - (Z_ẇ - X_u̇)·u·r   (complex, see Fossen)
F_z_added = (Y_v̇ - X_u̇)·v·p - ... (similar)
```

**Simplified for torpedo (assume symmetry Y_v̇ ≈ Z_ẇ):**
```
F_y_added_approx = -(X_u̇ - Y_v̇)·u·r + (Z_ẇ - Y_v̇)·w·p
F_z_added_approx = (Y_v̇ - X_u̇)·v·p - (Y_v̇ - Z_ẇ)·v·q
```

**(Much simpler if Y_v̇ = Z_ẇ, then many terms vanish.)**

### 4.2 Implementation

**File:** `src/physics/vehicle_dynamics.py`

**Add new method:**
```python
def compute_coriolis_forces(self, state: VehicleState) -> np.ndarray:
    """
    Compute complete Coriolis and centripetal forces C(ν)·ν.
    
    Implements Fossen (2011) Eq. 6.43 for rigid-body and added mass effects.
    Returns 6-element array: [F_x, F_y, F_z, L, M, N] in body frame.
    
    Args:
        state: Current vehicle state with velocities
        
    Returns:
        Coriolis force/moment vector [F_surge, F_sway, F_heave, L_roll, M_pitch, N_yaw]
    """
    u, v, w = state.velocity
    p, q, r = state.angular_velocity
    
    # ===== RIGID-BODY CORIOLIS =====
    # Linear velocity coupling: F = m × (ω × v)
    F_x_rb = self.mass * (v*r - w*q)
    F_y_rb = self.mass * (w*p - u*r)
    F_z_rb = self.mass * (u*q - v*p)
    
    # Angular velocity coupling: M = I × (ω × ω)
    # For diagonal inertia I = diag([I_xx, I_yy, I_zz]):
    L_rb = (self.I_zz - self.I_yy) * q * r
    M_rb = (self.I_xx - self.I_zz) * p * r
    N_rb = (self.I_yy - self.I_xx) * p * q
    
    # ===== ADDED MASS CORIOLIS =====
    # Added mass parameters
    X_u = self.added_mass_surge
    Y_v = self.added_mass_sway
    Z_w = self.added_mass_heave
    K_p = self.added_inertia_roll
    M_q = self.added_inertia_pitch
    N_r = self.added_inertia_yaw
    
    # Forces (Fossen Eq. 6.50, simplified for diagonal M_A)
    # Sway-yaw coupling
    F_x_a = 0.0  # No surge coupling for diagonal M_A
    F_y_a = -(X_u - Y_v) * u * r + (Z_w - Y_v) * w * p
    F_z_a = (Y_v - X_u) * v * p - (Y_v - Z_w) * v * q
    
    # Moments (angular coupling)
    L_a = (N_r - M_q) * q * r
    M_a = (K_p - N_r) * p * r
    N_a = (M_q - K_p) * p * q
    
    # ===== TOTAL CORIOLIS =====
    coriolis_forces = np.array([
        F_x_rb + F_x_a,
        F_y_rb + F_y_a,
        F_z_rb + F_z_a,
        L_rb + L_a,
        M_rb + M_a,
        N_rb + N_a
    ])
    
    return coriolis_forces
```

**Update `compute_hydrodynamic_forces()` (line 132+):**
```python
def compute_hydrodynamic_forces(self, state, fin_angles, thrust, environment):
    forces = HydrodynamicForces()
    
    # ... (existing drag, fin, buoyancy calculations) ...
    
    # === ADD CORIOLIS/CENTRIPETAL FORCES (NEW) ===
    coriolis = self.compute_coriolis_forces(state)
    forces.surge_force += coriolis[0]
    forces.sway_force += coriolis[1]
    forces.heave_force += coriolis[2]
    forces.roll_moment += coriolis[3]
    forces.pitch_moment += coriolis[4]
    forces.yaw_moment += coriolis[5]
    
    # REMOVE simplified sway Coriolis (now included in full C(ν)):
    # DELETE: sway_coriolis_RB = -self.mass * r * u  # Now in compute_coriolis_forces()
    
    return forces
```

### 4.3 Validation Tests

**Test 4.1: Coriolis skew-symmetry (energy conservation)**
```python
def test_coriolis_skew_symmetric():
    """
    Coriolis matrix must be skew-symmetric: C + Cᵀ = 0.
    This ensures ν·C(ν)·ν = 0 (no work done by Coriolis forces).
    """
    config = load_config("config/config.yaml")
    dynamics = VehicleDynamics(config)
    
    state = VehicleState()
    state.velocity = np.array([2.0, 0.5, 0.2])
    state.angular_velocity = np.array([0.1, 0.2, 0.3])
    
    coriolis = dynamics.compute_coriolis_forces(state)
    nu = np.concatenate([state.velocity, state.angular_velocity])
    
    # Power: P = ν · C(ν) · ν should be zero (or numerically small)
    power_coriolis = np.dot(nu, coriolis)
    
    assert abs(power_coriolis) < 1e-6, f"Coriolis doing work: P={power_coriolis} (should be 0)"
```

**Test 4.2: Straight-line motion (no Coriolis)**
```python
def test_coriolis_zero_for_straight_motion():
    """Pure surge with no rotation should have zero Coriolis."""
    dynamics = VehicleDynamics(load_config("config/config.yaml"))
    
    state = VehicleState()
    state.velocity = np.array([5.0, 0.0, 0.0])  # Pure surge
    state.angular_velocity = np.array([0.0, 0.0, 0.0])  # No rotation
    
    coriolis = dynamics.compute_coriolis_forces(state)
    
    # All Coriolis terms should be zero
    np.testing.assert_array_almost_equal(coriolis, np.zeros(6), decimal=10)
```

**Test 4.3: Turn induces centripetal force**
```python
def test_coriolis_turn_centripetal():
    """Yaw rate with surge should induce sway Coriolis force (centripetal)."""
    dynamics = VehicleDynamics(load_config("config/config.yaml"))
    
    state = VehicleState()
    u = 3.0  # m/s surge
    r = 0.5  # rad/s yaw rate
    state.velocity = np.array([u, 0.0, 0.0])
    state.angular_velocity = np.array([0.0, 0.0, r])
    
    coriolis = dynamics.compute_coriolis_forces(state)
    
    # Sway Coriolis: F_y = -m·u·r (centripetal acceleration)
    expected_F_sway = -dynamics.mass * u * r  # -180 * 3 * 0.5 = -270 N
    
    assert abs(coriolis[1] - expected_F_sway) < 1.0  # Within 1N (added mass adds complexity)
```

**Test 4.4: Gyroscopic pitch-roll coupling**
```python
def test_coriolis_gyroscopic_coupling():
    """Pitch and roll rates should couple via gyroscopic moment."""
    dynamics = VehicleDynamics(load_config("config/config.yaml"))
    
    state = VehicleState()
    p = 0.2  # rad/s roll rate
    q = 0.3  # rad/s pitch rate
    state.velocity = np.array([0.0, 0.0, 0.0])
    state.angular_velocity = np.array([p, q, 0.0])
    
    coriolis = dynamics.compute_coriolis_forces(state)
    
    # Yaw moment: N = (I_yy - I_xx) · p · q
    I_diff = dynamics.I_yy - dynamics.I_xx  # 175.6 - 2.3 = 173.3
    expected_N_yaw = I_diff * p * q  # 173.3 * 0.2 * 0.3 = 10.4 N·m
    
    assert abs(coriolis[5] - expected_N_yaw) < 0.5  # Within 0.5 N·m
```

### 4.4 Expected Behavior Changes

**Before:**
- No velocity/angular rate coupling
- Turns don't induce lateral forces (unrealistic)
- Pitch-roll motions independent (no gyroscopic effects)
- Energy not conserved (Coriolis missing)

**After:**
- Realistic turn dynamics with centripetal forces
- Pitch-roll-yaw coupling (gyroscopic moments)
- Energy conserved (Coriolis does no work: νᵀC(ν)ν = 0)
- More accurate maneuvering behavior

**Performance impact:** Low (6 additional force/moment calculations, simple algebraic operations)

**Risk:** High (complex coupling, potential for tuning issues)

**Validation critical:** Must verify energy conservation (Test 4.1) before deployment

### 4.5 Commit Message

```
feat(physics): implement full Coriolis/centripetal forces C(ν)ν

- Add compute_coriolis_forces() with rigid-body and added mass terms
- Implement velocity-angular rate coupling (centripetal forces)
- Implement gyroscopic moments (pitch-roll-yaw coupling)
- Add energy conservation test (Coriolis must be skew-symmetric)
- Update documentation with mathematical formulation

Physics: Completes the canonical equation M·ν̇ + C(ν)·ν + D(ν)·ν + g(η) = τ
Previously: C(ν) completely missing, violating energy conservation
Now: Full 6×6 Coriolis matrix implemented per Fossen (2011)

Refs: Physics-Audit.md §2.2.3 (ROOT CAUSE #3)
```

---

## PATCH 5: Restoring Forces from Geometry

**Objective:** Replace ad-hoc buoyancy with proper g(η) from CG/CB geometry

**Root Cause Addressed:** RC#7 (§2.4.3 of audit)

### 5.1 Mathematical Formulation

**Restoring force vector g(η) (Fossen Eq. 4.15):**

For a submerged body (no free surface):

```
g(η) = [ (W - B) · sin(θ)                                           ]
       [ -(W - B) · cos(θ) · sin(φ)                                 ]
       [ -(W - B) · cos(θ) · cos(φ)                                 ]
       [ -(y_g·W - y_b·B)·cos(θ)·cos(φ) + (z_g·W - z_b·B)·cos(θ)·sin(φ) ]
       [ (z_g·W - z_b·B)·sin(θ) + (x_g·W - x_b·B)·cos(θ)·cos(φ)    ]
       [ -(x_g·W - x_b·B)·cos(θ)·sin(φ) - (y_g·W - y_b·B)·sin(θ)   ]

where:
  W = m · g (weight)
  B = ρ_water · V_displaced · g (buoyancy)
  (x_g, y_g, z_g) = CG position in body frame
  (x_b, y_b, z_b) = CB position in body frame
```

**For torpedo with symmetry (y_g = y_b = 0, x_g = x_b = 0):**

```
g(η) = [ (W - B) · sin(θ)                ]
       [ 0                                ]
       [ -(W - B) · cos(θ) · cos(φ)      ]
       [ (z_g·W - z_b·B) · cos(θ) · sin(φ) ]  ← Roll restoring
       [ (z_g·W - z_b·B) · sin(θ)       ]  ← Pitch restoring
       [ 0                                ]
```

**Static equilibrium:** At rest, g(η_eq) = 0

**Trim angles (small-angle approximation):**
```
φ_trim = 0 (symmetric)
θ_trim = arcsin((B - W) / B)  ≈ (B - W) / B  for B ≈ W
```

### 5.2 Configuration Additions

**File:** `config/config.yaml`

```yaml
vehicle:
  # Existing geometry...
  mass: 180.0  # [kg]
  length: 3.25
  diameter: 0.32
  
  # Centers (body frame, origin at geometric center)
  cg_offset_x: 0.0      # [m] Longitudinal CG position (0 = at center)
  cg_offset_y: 0.0      # [m] Lateral (should be 0 for symmetric vehicle)
  cg_offset_z: -0.02    # [m] Vertical (negative = below centerline)
  
  cb_offset_x: 0.0      # [m] CB longitudinal
  cb_offset_y: 0.0      # [m] CB lateral
  cb_offset_z: 0.0      # [m] CB vertical (0 = at geometric center)
  
  # Buoyancy specification
  buoyancy_method: "displaced_volume"  # "displaced_volume" or "buoyancy_fraction"
  displaced_volume: 0.179  # [m³] Physical volume (if method = displaced_volume)
  buoyancy_fraction: 1.02  # [-] B/W ratio (if method = buoyancy_fraction)
  
  # Alternatively, let config calculate from mass:
  # V_neutral = mass / fluid_density = 180 / 1025 = 0.1756 m³
  # V_actual = V_neutral * buoyancy_fraction

environment:
  fluid_density: 1025.0  # [kg/m³]
  gravity: 9.81          # [m/s²]
```

**Choice of buoyancy method:**
- **displaced_volume:** Specify physical volume directly (from CAD model)
- **buoyancy_fraction:** Specify B/W ratio (e.g., 1.02 = 2% positive buoyancy)

### 5.3 Implementation

**File:** `src/physics/vehicle_dynamics.py`

**Update initialization:**
```python
def __init__(self, config):
    # ... (existing) ...
    
    # Center positions (body frame)
    self.r_g = np.array([
        vehicle_cfg['cg_offset_x'],
        vehicle_cfg['cg_offset_y'],
        vehicle_cfg['cg_offset_z']
    ])  # [m] CG position
    
    self.r_b = np.array([
        vehicle_cfg['cb_offset_x'],
        vehicle_cfg['cb_offset_y'],
        vehicle_cfg['cb_offset_z']
    ])  # [m] CB position
    
    # Buoyancy calculation
    buoyancy_method = vehicle_cfg.get('buoyancy_method', 'buoyancy_fraction')
    
    if buoyancy_method == 'displaced_volume':
        V_displaced = vehicle_cfg['displaced_volume']  # [m³]
    elif buoyancy_method == 'buoyancy_fraction':
        buoyancy_fraction = vehicle_cfg['buoyancy_fraction']  # B/W ratio
        V_neutral = self.mass / self.fluid_density
        V_displaced = V_neutral * buoyancy_fraction
    else:
        raise ValueError(f"Unknown buoyancy_method: {buoyancy_method}")
    
    self.displaced_volume = V_displaced
    self.weight = self.mass * self.gravity  # [N]
    self.buoyancy = self.fluid_density * V_displaced * self.gravity  # [N]
    
    # Log buoyancy status
    buoyancy_percent = (self.buoyancy / self.weight - 1.0) * 100
    logger.info(f"Buoyancy: W={self.weight:.1f}N, B={self.buoyancy:.1f}N "
                f"({buoyancy_percent:+.1f}% buoyancy)")
    
    # Validate CG/CB offsets
    if abs(self.r_g[1]) > 1e-6 or abs(self.r_b[1]) > 1e-6:
        logger.warning(f"Non-zero lateral CG/CB offsets: may cause roll bias")
```

**Add restoring force method:**
```python
def compute_restoring_forces(self, state: VehicleState) -> np.ndarray:
    """
    Compute gravitational and buoyancy restoring forces g(η).
    
    Implements Fossen (2011) Eq. 4.15 for submerged vehicle.
    
    Args:
        state: Current vehicle state (needs orientation)
        
    Returns:
        Restoring force/moment vector [F_x, F_y, F_z, L, M, N]
    """
    phi, theta, psi = state.orientation
    
    # Trigonometric functions
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    
    W = self.weight
    B = self.buoyancy
    
    x_g, y_g, z_g = self.r_g
    x_b, y_b, z_b = self.r_b
    
    # Forces (Fossen Eq. 4.15)
    F_x = (W - B) * s_theta
    F_y = -(W - B) * c_theta * s_phi
    F_z = -(W - B) * c_theta * c_phi
    
    # Moments (lever arms × forces)
    # Roll: L = (z_g·W - z_b·B)·cos(θ)·sin(φ) - (y_g·W - y_b·B)·cos(θ)·cos(φ)
    L = (z_g*W - z_b*B) * c_theta * s_phi - (y_g*W - y_b*B) * c_theta * c_phi
    
    # Pitch: M = (z_g·W - z_b·B)·sin(θ) + (x_g·W - x_b·B)·cos(θ)·cos(φ)
    M = (z_g*W - z_b*B) * s_theta + (x_g*W - x_b*B) * c_theta * c_phi
    
    # Yaw: N = -(x_g·W - x_b·B)·cos(θ)·sin(φ) - (y_g·W - y_b·B)·sin(θ)
    N = -(x_g*W - x_b*B) * c_theta * s_phi - (y_g*W - y_b*B) * s_theta
    
    return np.array([F_x, F_y, F_z, L, M, N])
```

**Update `compute_hydrodynamic_forces()`:**
```python
def compute_hydrodynamic_forces(self, state, fin_angles, thrust, environment):
    forces = HydrodynamicForces()
    
    # ... (existing: drag, fins, Coriolis) ...
    
    # === ADD PROPER RESTORING FORCES (REPLACE AD-HOC) ===
    restoring = self.compute_restoring_forces(state)
    forces.surge_force += restoring[0]
    forces.sway_force += restoring[1]
    forces.heave_force += restoring[2]
    forces.roll_moment += restoring[3]
    forces.pitch_moment += restoring[4]
    forces.yaw_moment += restoring[5]
    
    # DELETE old ad-hoc buoyancy code:
    # buoyancy_force = -self.mass * self.gravity * 0.02  # REMOVE
    # restoring_pitch_moment = -0.5 * self.mass * self.gravity * 0.02 * np.sin(pitch)  # REMOVE
    
    return forces
```

### 5.4 Validation Tests

**Test 5.1: Neutral buoyancy equilibrium**
```python
def test_neutral_buoyancy_trim():
    """Neutral buoyancy (W=B) should have zero net force at zero angles."""
    config = load_config("config/config.yaml")
    config['vehicle']['buoyancy_fraction'] = 1.0  # Exactly neutral
    
    dynamics = VehicleDynamics(config)
    
    state = VehicleState()
    state.orientation = np.array([0.0, 0.0, 0.0])  # Level
    
    restoring = dynamics.compute_restoring_forces(state)
    
    # All forces should be zero (W = B, no angle)
    np.testing.assert_array_almost_equal(restoring, np.zeros(6), decimal=8)
```

**Test 5.2: Positive buoyancy ascent force**
```python
def test_positive_buoyancy_ascent():
    """Positive buoyancy should create upward force at level trim."""
    config = load_config("config/config.yaml")
    config['vehicle']['buoyancy_fraction'] = 1.02  # 2% positive
    
    dynamics = VehicleDynamics(config)
    
    state = VehicleState()
    state.orientation = np.array([0.0, 0.0, 0.0])
    
    restoring = dynamics.compute_restoring_forces(state)
    
    # Heave force should be negative (upward in NED body frame)
    # F_z = -(W - B)·cos(0)·cos(0) = -(W - 1.02W)·1 = 0.02W
    expected_F_z = -(dynamics.weight - dynamics.buoyancy)  # Negative (upward)
    
    assert restoring[2] < 0, "Should have upward force"
    assert abs(restoring[2] - expected_F_z) < 0.1
```

**Test 5.3: Roll restoring moment**
```python
def test_roll_restoring_moment():
    """CG below CB should create roll restoring moment (stable)."""
    config = load_config("config/config.yaml")
    # CG below CB: z_g = -0.02 m, z_b = 0.0 m
    
    dynamics = VehicleDynamics(config)
    
    state = VehicleState()
    state.orientation = np.array([np.deg2rad(10), 0.0, 0.0])  # 10° roll
    
    restoring = dynamics.compute_restoring_forces(state)
    
    # Roll moment L should oppose roll (restoring)
    # L = (z_g·W - z_b·B)·cos(0)·sin(10°)
    # z_g < z_b → negative coefficient → negative L for positive φ (restoring)
    assert restoring[3] < 0, "Roll moment should restore toward level"
```

**Test 5.4: Pitch restoring moment**
```python
def test_pitch_restoring_moment():
    """Positive buoyancy with CG below CB should restore pitch."""
    config = load_config("config/config.yaml")
    config['vehicle']['buoyancy_fraction'] = 1.02
    
    dynamics = VehicleDynamics(config)
    
    state = VehicleState()
    state.orientation = np.array([0.0, np.deg2rad(15), 0.0])  # 15° pitch (nose up)
    
    restoring = dynamics.compute_restoring_forces(state)
    
    # Pitch moment M should oppose pitch (restoring)
    # M = (z_g·W - z_b·B)·sin(θ)
    # Since z_g < z_b and B > W: (z_g·W - z_b·B) < 0
    # θ > 0 → M < 0 (restoring)
    assert restoring[4] < 0, "Pitch moment should restore toward level"
```

### 5.5 Expected Behavior Changes

**Before:**
- Ad-hoc buoyancy (magic 2%, no geometry basis)
- Incorrect restoring moments (factor 0.5 error)
- No roll restoring (vehicle unstable in roll)

**After:**
- Physically correct buoyancy from displaced volume
- Accurate restoring moments from CG/CB lever arms
- Roll stability from vertical CG-CB offset
- Predictable trim angles

**Performance impact:** Negligible (trigonometric evaluations, same as before)

**Risk:** Low (well-established hydrostatic theory)

### 5.6 Commit Message

```
feat(physics): implement proper hydrostatic restoring forces g(η)

- Replace ad-hoc buoyancy with g(η) from Fossen (2011) Eq. 4.15
- Calculate buoyancy from displaced volume or buoyancy fraction
- Implement roll/pitch/yaw restoring moments from CG-CB geometry
- Add config options for buoyancy specification method
- Add tests for static equilibrium, trim angles, and restoring stability

Previously: Magic numbers (2% buoyancy, 0.5 restoring factor)
Now: Physically derived from CG/CB positions and displaced volume

Refs: Physics-Audit.md §2.4.3 (ROOT CAUSE #7)
```

---

## PATCH 6: Heave Dynamics & Velocity Transformation

**Objective:** Fix heave dynamics confusion and properly transform velocities

**Root Cause Addressed:** RC#5 (§2.1.5 of audit)

*(Implementation details similar structure to Patches 1-5...)*

**Key change:** Separate heave acceleration in body frame from position integration in NED frame using rotation matrix.

---

## PATCH 7: Current Modeling (Relative Velocity)

**Objective:** Fix current to use relative velocity, not external force

**Root Cause Addressed:** §2.8.1 of audit

*(Implementation: Transform current to body frame, compute all hydrodynamic forces on relative velocity...)*

---

## PATCH 8: Control Improvements (Depth PID, Gain Scheduling)

**Objective:** Add depth integrator, improve low-speed control

**Root Cause Addressed:** §2.6.2, §2.6.3 of audit

*(Add PID for depth instead of pure P, velocity-dependent gain scaling...)*

---

## PATCH 9: Numerical Upgrades (RK4, Energy Monitoring)

**Objective:** Upgrade integrator and add energy diagnostics

**Root Cause Addressed:** §2.7.1, §2.7.3 of audit

*(Optional: Implement RK4, mandatory: add energy logging...)*

---

## PATCH 10: Documentation & Validation Suite

**Objective:** Complete documentation and create standard test suite

**Deliverables:**
- `ValidationPlan.md` with all tests
- Unit tests for each physics component
- Integration test scenarios
- Regression comparison plots
- Updated README with correct physics claims

---

# IMPLEMENTATION TIMELINE

**Estimated effort:** 4-6 weeks for experienced researcher

| Patch | Effort | Risk | Priority | Dependencies |
|-------|--------|------|----------|--------------|
| 1. Euler kinematics | 1 week | Low | Critical | None |
| 2. Mass/inertia | 3 days | Low | High | None |
| 3. Sway dynamics | 1 week | Medium | High | Patch 2 |
| 4. Coriolis forces | 2 weeks | High | High | Patches 2, 3 |
| 5. Restoring forces | 1 week | Low | High | None |
| 6. Heave fix | 1 week | Medium | High | Patch 1 |
| 7. Current model | 3 days | Low | Medium | Patches 3, 6 |
| 8. Control improvements | 1 week | Medium | Medium | Patches 1-7 |
| 9. Numerical upgrades | 1 week | Low | Optional | All above |
| 10. Documentation | 1 week | Low | Critical | All above |

**Critical path:** 1 → 2 → 3 → 4 → Validation

**Minimum viable fix:** Patches 1, 2, 5 (frame consistency + restoring forces)

**Full physics correctness:** Patches 1-7

---

# TESTING & VALIDATION STRATEGY

## Unit Tests (per patch)
- Test each new function in isolation
- Verify mathematical properties (symmetry, energy conservation)
- Check edge cases and singularities

## Integration Tests (after each patch)
- Run full simulation with standard scenario
- Compare before/after plots
- Check for unexpected side effects
- Verify stability (no divergence)

## Regression Tests
- Maintain golden data from before patches
- Automated comparison of key metrics
- Alert on significant deviations

## Energy Budget Validation
```python
# Add to data logger
energy_kinetic = 0.5 * m * ||v||² + 0.5 * I · ||ω||²
energy_potential = -m * g * z
energy_total = energy_kinetic + energy_potential

# Log: time, E_kin, E_pot, E_total, P_thrust, P_damping
# Verify: dE/dt = P_thrust - P_damping (within numerical error)
```

## Physical Law Checks
1. **Momentum conservation** (no external forces)
2. **Angular momentum conservation** (no external moments)
3. **Energy dissipation** (total energy decreases with damping)
4. **Buoyancy equilibrium** (static trim matches theory)

---

**End of Change Plan**

**Next Steps:**
1. Review this plan with team/advisor
2. Set up development branch: `git checkout -b physics-fixes-v2`
3. Implement Patch 1 (Euler kinematics)
4. Run validation tests
5. Iterate

---

**Contact for questions:** Refer to Physics-Audit.md for detailed technical background on each issue.

