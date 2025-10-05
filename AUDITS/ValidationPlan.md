# AUV 6-DOF Physics Validation Plan
## Test Suite for Before/After Physics Corrections

**Date:** October 5, 2025  
**Purpose:** Define measurable, repeatable tests to validate physics corrections  
**Prerequisites:** `Physics-Audit.md` and `ChangePlan.md` reviewed  
**Target:** Demonstrate correctness of corrected physics model

---

# VALIDATION PHILOSOPHY

## Core Principles

1. **Test fundamental physics laws** (conservation, equilibrium, causality)
2. **Compare against analytical solutions** where available
3. **Measure before/after** to quantify improvements
4. **Use simple scenarios** (isolate effects, avoid confounding variables)
5. **Define pass/fail criteria** (objective, quantitative thresholds)

## Validation Hierarchy

```
Level 1: Unit Tests
  ↓ (mathematical correctness of individual functions)
Level 2: Component Tests  
  ↓ (physics subsystems: damping, restoring, Coriolis)
Level 3: Integration Tests
  ↓ (full simulation, multiple effects combined)
Level 4: Regression Tests
  ↓ (compare against golden data or literature)
```

---

# LEVEL 1: UNIT TESTS

## Test Suite 1.1: Kinematic Transformations

### Test 1.1.1: Euler Rate Transformation Identity

**Objective:** Verify that at zero Euler angles, transformation T is identity matrix

**Setup:**
```python
euler_angles = [0, 0, 0]  # rad
body_rates = [0.1, 0.2, 0.3]  # rad/s
```

**Expected:**
```
euler_rates = T(0,0,0) @ body_rates = [0.1, 0.2, 0.3]  # Identity transformation
```

**Pass criteria:** `||euler_rates - body_rates|| < 1e-10`

**Rationale:** T(0,0,0) = I by definition (Fossen Eq. 2.29 at η=0)

---

### Test 1.1.2: Gimbal Lock Detection

**Objective:** Verify warning issued when approaching singularity

**Setup:**
```python
euler_angles = [0, 85°, 0]  # Near 90° pitch
body_rates = [0.1, 0.1, 0.1]
```

**Expected:** RuntimeWarning logged containing "gimbal lock"

**Pass criteria:** Warning emitted, computation does not crash

**Rationale:** User must be informed of approaching singularity

---

### Test 1.1.3: Yaw-Roll Coupling at Pitch

**Objective:** Verify that non-zero pitch couples yaw rate into roll rate

**Setup:**
```python
euler_angles = [0°, 30°, 0°]
body_rates = [0, 0, 1.0]  # Pure yaw rate r=1.0 rad/s
```

**Expected:**
```
phi_dot = 1.0 * cos(0) * tan(30°) = 0.577 rad/s  (yaw couples into roll)
theta_dot = -1.0 * sin(0) = 0
psi_dot = 1.0 / cos(30°) = 1.155 rad/s
```

**Pass criteria:** 
- `|phi_dot - 0.577| < 0.001`
- `|psi_dot - 1.155| < 0.001`

**Rationale:** Validates T matrix elements correct (Fossen Table 2.1)

---

## Test Suite 1.2: Coriolis Matrix Properties

### Test 1.2.1: Skew-Symmetry (Energy Conservation)

**Objective:** Verify C(ν) satisfies `νᵀ·C(ν)·ν = 0` (Coriolis does no work)

**Setup:**
```python
velocities = [2.0, 0.5, 0.2]  # m/s
angular_rates = [0.1, 0.2, 0.3]  # rad/s
nu = concatenate([velocities, angular_rates])
```

**Expected:**
```python
C_nu = compute_coriolis_forces(state)
power = dot(nu, C_nu)
power ≈ 0  (within numerical precision)
```

**Pass criteria:** `|power| < 1e-6 W`

**Rationale:** Fundamental property of Coriolis (Fossen Th. 3.2)

---

### Test 1.2.2: Straight-Line Motion (No Coriolis)

**Objective:** Pure surge with no rotation → all Coriolis terms zero

**Setup:**
```python
state.velocity = [5.0, 0, 0]  # Pure surge
state.angular_velocity = [0, 0, 0]  # No rotation
```

**Expected:** `C(ν) = 0₆` (all elements zero)

**Pass criteria:** `||C(ν)|| < 1e-12`

**Rationale:** Coriolis only appears with velocity/angular rate coupling

---

### Test 1.2.3: Centripetal Force in Turn

**Objective:** Yaw with surge induces sway Coriolis (centripetal force)

**Setup:**
```python
u = 3.0 m/s  (surge)
r = 0.5 rad/s  (yaw rate, turning)
v = 0  (initially no sway)
```

**Expected:**
```python
F_sway_coriolis = -m * u * r = -180 * 3.0 * 0.5 = -270 N (inward, toward turn center)
```

**Pass criteria:** `|F_sway - (-270)| < 10 N` (within 10N due to added mass terms)

**Rationale:** Classic centripetal acceleration a = v²/R = u·r

---

## Test Suite 1.3: Restoring Forces

### Test 1.3.1: Neutral Buoyancy Equilibrium

**Objective:** W = B and level attitude → zero net restoring force

**Setup:**
```python
config['vehicle']['buoyancy_fraction'] = 1.0  # Neutral
state.orientation = [0, 0, 0]  # Level
```

**Expected:** `g(η) = 0₆` (all restoring forces/moments zero)

**Pass criteria:** `||g(η)|| < 1e-8`

**Rationale:** Definition of neutral buoyancy at level trim

---

### Test 1.3.2: Positive Buoyancy Ascent Force

**Objective:** B > W creates upward force at level attitude

**Setup:**
```python
config['vehicle']['buoyancy_fraction'] = 1.02  # 2% positive
state.orientation = [0, 0, 0]
```

**Expected:**
```python
F_z = -(W - B) * cos(0) * cos(0) = -(180*9.81 - 1.02*180*9.81) = -35.3 N (upward)
```

**Pass criteria:** 
- `F_z < 0` (upward in NED body frame)
- `|F_z - (-35.3)| < 1.0 N`

**Rationale:** Net upward force causes ascent

---

### Test 1.3.3: Roll Restoring Moment (Metacentric Stability)

**Objective:** CG below CB creates roll restoring moment

**Setup:**
```python
config['vehicle']['cg_offset_z'] = -0.02 m  # CG below centerline
config['vehicle']['cb_offset_z'] = 0.0 m   # CB at centerline
state.orientation = [10°, 0, 0]  # 10° roll (starboard down)
```

**Expected:**
```python
L_roll = (z_g*W - z_b*B) * cos(0) * sin(10°)
       = (-0.02*1765 - 0*1800) * 1.0 * 0.174
       = -35.3 * 0.174 = -6.1 N·m (restoring toward level)
```

**Pass criteria:**
- `L_roll < 0` (opposes positive roll)
- `|L_roll - (-6.1)| < 1.0 N·m`

**Rationale:** Validates metacentric stability (Fossen §4.2.1)

---

### Test 1.3.4: Pitch Restoring Moment

**Objective:** Positive buoyancy with vertical CG-CB offset restores pitch

**Setup:**
```python
config['vehicle']['buoyancy_fraction'] = 1.02
state.orientation = [0, 15°, 0]  # 15° pitch (nose up)
```

**Expected:**
```python
M_pitch = (z_g*W - z_b*B) * sin(15°)
        = (-0.02*1765 - 0*1800) * 0.259
        = -35.3 * 0.259 = -9.1 N·m (nose-down restoring)
```

**Pass criteria:**
- `M_pitch < 0` (opposes positive pitch)
- `|M_pitch - (-9.1)| < 1.5 N·m`

**Rationale:** Pitch stability from buoyancy-weight couple

---

# LEVEL 2: COMPONENT TESTS

## Test Suite 2.1: Damping Dissipation

### Test 2.1.1: Surge Velocity Decay

**Objective:** Pure surge with no thrust decays exponentially

**Setup:**
```python
Initial: u = 2.0 m/s, thrust = 0
Run: 60 seconds
```

**Expected:**
Exponential decay: `u(t) = u₀ · exp(-d₂·u₀·t / (m + X_u̇))`

Time constant: `τ ≈ (m + X_u̇) / (d₂·u₀) = 189 / (35·2.0) = 2.7 s`

At t=10s: `u ≈ 2.0 · exp(-10/2.7) = 0.055 m/s`

**Pass criteria:** 
- `0.01 < u(10s) < 0.10 m/s`
- Monotonic decrease (du/dt < 0 always)

**Rationale:** Damping must dissipate energy, not add it

---

### Test 2.1.2: Roll Rate Decay

**Objective:** Initial roll rate decays due to damping

**Setup:**
```python
Initial: p = 20°/s = 0.349 rad/s, no fins
Run: 30 seconds
```

**Expected:**
Linear+quadratic damping: `I·ṗ = -c_p·p - c_p_quad·p·|p|`

Time constant (initial): `τ ≈ I_total / c_p = 2.53 / 2.0 = 1.27 s`

At t=5s: `p ≈ 0.349 · exp(-5/1.27) ≈ 0.0055 rad/s ≈ 0.3°/s`

**Pass criteria:**
- `|p(5s)| < 1°/s`
- Monotonic decay
- Energy decreases: E_rot(t) < E_rot(0)

**Rationale:** Validates roll damping magnitude

---

### Test 2.1.3: Sway Velocity with Damping

**Objective:** Lateral velocity decays when no lateral force applied

**Setup:**
```python
Initial: v = 1.0 m/s (pure sway), u = 0, all ω = 0
Run: 20 seconds
```

**Expected:**
Quadratic damping dominates: `(m + Y_v̇)·v̇ = -d₂_sway·v·|v|`

Quick initial decay (quadratic), then slows

At t=5s: `v < 0.3 m/s` (estimate, needs numerical integration)

**Pass criteria:**
- `v(5s) < 0.5 m/s`
- `v(20s) < 0.1 m/s`
- Monotonic decay

**Rationale:** Verifies cross-flow damping active

---

## Test Suite 2.2: Static Trim & Equilibrium

### Test 2.2.1: Find Static Trim Angle

**Objective:** Vehicle settles to equilibrium orientation at rest

**Setup:**
```python
Initial: at rest, u=v=w=0, ω=0, any initial orientation
Thrust: 0
Fins: 0
Run: 120 seconds (long enough to settle)
```

**Expected:**
Settles to trim where `g(η_trim) = 0`:

For buoyancy_fraction = 1.02 (2% positive):
```
θ_trim ≈ arcsin((B - W)/B) = arcsin(0.02) = 1.15° (nose up)
φ_trim = 0° (symmetric)
```

**Pass criteria:**
- `|φ_final| < 0.5°`
- `0.5° < θ_final < 2.0°` (within expected range)
- `||ω_final|| < 0.01 rad/s` (essentially stopped)
- Net forces < 1N, net moments < 0.1 N·m

**Rationale:** Fundamental static equilibrium test

---

### Test 2.2.2: Restoring Stiffness (Small Angle)

**Objective:** Small perturbation from trim creates restoring moment

**Setup:**
```python
Start at trim (from Test 2.2.1)
Apply: Δφ = +5° (small roll disturbance)
Release (no control)
```

**Expected:**
Oscillation with period: `T = 2π·sqrt(I_total / k_restoring)`

Where `k_restoring = (z_g*W - z_b*B) * cos(θ_trim)` for roll

**Pass criteria:**
- Vehicle oscillates about φ=0
- Amplitude decreases (damped oscillation)
- Period within 20% of predicted

**Rationale:** Validates linear restoring stiffness

---

## Test Suite 2.3: Turn Dynamics

### Test 2.3.1: Coordinated Turn (Coriolis Verification)

**Objective:** During steady turn, centripetal force balanced by sway

**Setup:**
```python
Constant: u = 2.0 m/s surge, r = 0.3 rad/s yaw rate (steady turn)
Run: 30 seconds
Control: maintain u and r constant via thrust and yaw moment
```

**Expected:**
Steady-state sway velocity develops:
```
0 = -m·u·r - d₂_sway·v·|v|  (equilibrium)
v_ss ≈ -sqrt(m·u·r / d₂_sway) = -sqrt(180*2*0.3 / 120) ≈ -1.0 m/s (inward)
```

Turn radius: `R = u / r = 2.0 / 0.3 = 6.7 m`

**Pass criteria:**
- Sway velocity stabilizes: `|v - v_ss| < 0.2 m/s` after 10s
- Path curvature: `1/R = r/u` within 10%
- Vehicle follows circular arc in NED frame

**Rationale:** Validates Coriolis coupling u-r-v

---

## Test Suite 2.4: Actuator Response

### Test 2.4.1: Thrust Lag Dynamics

**Objective:** Thrust command produces first-order response

**Setup:**
```python
Initial: thrust = 0
Command: step to 100 N at t=0
Time constant: τ = 0.15 s (config)
```

**Expected:**
```python
T(t) = 100 * (1 - exp(-t/0.15))
T(0.15s) = 63.2 N  (63% of final)
T(0.45s) = 95.0 N  (95% of final)
```

**Pass criteria:**
- `60 < T(0.15s) < 66 N`
- `93 < T(0.45s) < 97 N`

**Rationale:** Validates actuator dynamics model

---

### Test 2.4.2: Fin Allocation Consistency

**Objective:** Desired moments map correctly to fin deflections

**Setup:**
```python
Desired: [L=10 N·m, M=20 N·m, N=5 N·m]
Speed: u = 2.0 m/s
```

**Expected:**
Allocation produces fin angles, when applied to physics, reproduce moments

**Pass criteria:**
- Moments from fins within 5% of desired
- Fin deflections within limits (±25°)
- Symmetric patterns for symmetric moments

**Rationale:** Validates control allocation matrix

---

# LEVEL 3: INTEGRATION TESTS

## Test Suite 3.1: Free Decay Tests

### Test 3.1.1: Roll Free Decay

**Objective:** Measure roll damping from free oscillation

**Setup:**
```python
Initial: φ = 15°, all else zero
Release: no control, no thrust
Run: 60 seconds
```

**Expected:**
Damped oscillation with:
- Natural frequency: `ω_n = sqrt(k_restoring / I_total)`
- Damping ratio: `ζ = c_roll / (2·sqrt(k_restoring·I_total))`

**Pass criteria:**
- Energy monotonically decreases
- Overshoot on first oscillation < 30%
- Settles to |φ| < 1° within 30s

**Measurements:**
- Period T_measured → compare with `T_theory = 2π/ω_n`
- Logarithmic decrement δ → estimate damping

**Rationale:** Classic stability test (Fossen §14.3.2)

---

### Test 3.1.2: Pitch Free Decay

**Objective:** Measure pitch damping and restoring

**Setup:**
```python
Initial: θ = 10°, all else zero
Release: no control
Run: 60 seconds
```

**Expected:**
Similar to roll but:
- Restoring from buoyancy couple
- Higher inertia → longer period

**Pass criteria:**
- Oscillation about trim angle (~1°, not 0°)
- Energy decreases monotonically
- Settles within 45s

**Rationale:** Pitch dynamics validation

---

### Test 3.1.3: Yaw Free Decay

**Objective:** Measure yaw damping (no restoring for symmetric vehicle)

**Setup:**
```python
Initial: ψ = 0°, r = 10°/s yaw rate
Release: no control
Run: 60 seconds
```

**Expected:**
Exponential decay (no restoring, only damping):
```
r(t) = r₀ · exp(-c_r / I_total · t)
```

**Pass criteria:**
- Yaw rate decays to |r| < 0.5°/s within 30s
- Heading drifts (no restoring) but stabilizes
- Energy decreases

**Rationale:** Pure damping test (no restoring stiffness)

---

## Test Suite 3.2: Step Response Tests

### Test 3.2.1: Speed Step Response

**Objective:** Command speed change, measure settling time

**Setup:**
```python
Initial: level trim, u = 0
Command: u_desired = 1.5 m/s
Control: active speed controller
Run: 60 seconds
```

**Expected:**
- Rise time: t_rise ~ 5-10 s
- Overshoot: < 10%
- Settling time (±2%): < 20 s
- Steady-state error: < 0.05 m/s

**Pass criteria:**
- Reaches 1.4-1.6 m/s
- No oscillations (well-damped)
- Stable

**Rationale:** Validates speed control loop

---

### Test 3.2.2: Depth Step Response

**Objective:** Depth command via pitch control

**Setup:**
```python
Initial: surface (z=0), level
Command: depth = 10 m
Speed: u = 1.5 m/s constant
Control: active depth→pitch→fins
Run: 120 seconds
```

**Expected:**
- Pitch angles out (nose down) ~10-15°
- Descends smoothly
- Reaches 10m depth
- Levels out at target

**Pass criteria:**
- Final depth: 9.5 < z < 10.5 m
- Overshoot: < 1 m
- Settling time: < 60 s
- No porpoising (oscillation)

**Rationale:** Depth control critical for AUV missions

---

### Test 3.2.3: Heading Step Response

**Objective:** Turn to new heading

**Setup:**
```python
Initial: ψ = 0°, straight line
Command: ψ_desired = 90° (turn East)
Speed: u = 1.5 m/s
Control: active heading controller
Run: 60 seconds
```

**Expected:**
- Yaw rate develops: r_max ~ 5-10°/s
- Turn arc visible in trajectory
- Settles at 90° ± 5°

**Pass criteria:**
- Final heading: 85° < ψ < 95°
- Turn radius: 10-30 m (reasonable)
- No overshoot > 10°
- Smooth turn (no jerking)

**Rationale:** Heading control fundamental to navigation

---

## Test Suite 3.3: Mission Scenarios

### Test 3.3.1: Dive-Turn-Surface Maneuver

**Objective:** Combined depth and heading changes

**Setup:**
```python
Phase 1 (0-30s): Dive to 10m, heading 0°
Phase 2 (30-60s): Turn to 180°, maintain depth
Phase 3 (60-90s): Surface to 2m, maintain heading
```

**Expected:**
- Smooth transitions between phases
- Depth maintained during turn
- Heading maintained during dive/surface

**Pass criteria:**
- All waypoints reached within tolerance
- No instabilities
- Energy budget makes sense (thrust work > 0)

**Rationale:** Realistic mission profile

---

### Test 3.3.2: Spiral Dive

**Objective:** Combined yaw and descent

**Setup:**
```python
Command: constant yaw rate r = 5°/s, descend to 20m
Duration: 120 seconds
```

**Expected:**
- Helical path (3D spiral)
- Reaches target depth
- Multiple complete turns (>2 full circles)

**Pass criteria:**
- Spiral visible in 3D trajectory plot
- Depth control accurate
- Turn radius consistent

**Rationale:** Tests coupling between DOFs

---

## Test Suite 3.4: Environmental Effects

### Test 3.4.1: Station-Keeping in Current

**Objective:** Maintain position in constant current

**Setup:**
```python
Current: 0.5 m/s from East (pushing vehicle West)
Command: hold position (0, 0) in NED
Control: active navigation, thrust, heading
Duration: 300 seconds
```

**Expected:**
- Vehicle crabs into current (heading East of North)
- Ground track near origin
- Thrust balances current drag

**Pass criteria:**
- Position drift < 20 m over 5 minutes
- Heading adjusts appropriately (not aligned with North)
- Stable (no oscillations)

**Rationale:** Validates current modeling via relative velocity

---

# LEVEL 4: REGRESSION & COMPARISON

## Test Suite 4.1: Energy Budget Validation

### Test 4.1.1: Energy Conservation (No Thrust, No Damping)

**Objective:** Total energy constant if no dissipation or input

**Setup:**
```python
Disable: damping (set all c=0, d=0)
Initial: u = 2 m/s, z = -10 m (depth 10m)
Thrust: 0
Run: 60 seconds
```

**Expected:**
```python
E_total(t) = E_kinetic + E_potential = constant
E_kin = 0.5 * m * u² + 0.5 * I * ω²
E_pot = -m * g * z
```

**Pass criteria:**
- `|E_total(t) - E_total(0)| < 1% * E_total(0)` for all t
- Drift < 0.1% per second (numerical error)

**Rationale:** Fundamental physics law check

---

### Test 4.1.2: Energy Dissipation (Damping Only)

**Objective:** Energy decreases monotonically with damping

**Setup:**
```python
Enable: damping
Initial: u = 2 m/s, at trim
Thrust: 0
Run: 60 seconds
```

**Expected:**
```python
dE/dt = -P_damping < 0  (always)
P_damping = d₂·u³ + c_p·p² + ... (all positive)
```

**Pass criteria:**
- `E(t) < E(0)` for all t > 0
- `dE/dt < 0` (verify numerically)
- Final energy near zero (dissipated)

**Rationale:** Thermodynamics (energy dissipates)

---

### Test 4.1.3: Energy Balance (Thrust + Damping)

**Objective:** Energy change equals work done

**Setup:**
```python
Constant thrust: T = 50 N
Duration: 60 seconds
```

**Expected:**
```python
ΔE_total = ∫ P_thrust dt - ∫ P_damping dt

where:
  P_thrust = T · u (power into system)
  P_damping = sum of all damping power
```

**Pass criteria:**
- `|ΔE_measured - ΔE_predicted| < 5% * ΔE_predicted`

**Rationale:** First law of thermodynamics

---

## Test Suite 4.2: Comparison with Simplified Models

### Test 4.2.1: 1-DOF Surge vs. Full Model

**Objective:** Pure surge matches simplified surge-only model

**Setup:**
```python
Constrain: v=w=0, φ=θ=ψ=0 (force DOFs inactive)
Initial: u = 0
Thrust: T(t) as desired
```

**Expected:**
Full model reduces to: `(m + X_u̇)·u̇ = T - d₂·u·|u|`

**Pass criteria:**
- Full model matches 1-DOF analytical solution
- Error < 0.1% in velocity

**Rationale:** Validates surge dynamics isolation

---

## Test Suite 4.3: Literature Comparison

### Test 4.3.1: Prestero (2001) REMUS Data

**Objective:** Compare against published REMUS simulation/experimental data

**Setup:**
Use mission profiles from Prestero's thesis:
- Depth step responses
- Heading changes
- Speed variations

**Expected:**
Qualitative similarity:
- Time constants within factor of 2
- Overshoot characteristics similar
- Stability margins comparable

**Pass criteria:**
- Behavior in same ballpark (not exact match, different parameters)
- No qualitative contradictions

**Rationale:** Benchmark against established work

---

# REGRESSION TEST PROTOCOL

## Baseline Capture (Before Fixes)

**Run all tests on current codebase:**
```bash
python -m pytest tests/validation/ --baseline --output=baseline_results.json
```

**Capture:**
- All test results (pass/fail)
- Numerical metrics (time constants, overshoots, energies)
- Plots of key trajectories
- Energy time histories

**Create golden data:** Store as `tests/golden/baseline_unfixed/`

## After Each Patch

**Run regression suite:**
```bash
python -m pytest tests/validation/ --compare-baseline=baseline_results.json
```

**Check:**
1. Previously passing tests still pass (no regressions)
2. Target tests now pass (fix effective)
3. Metrics improved (quantifiable)

**Update documentation:** Note which tests are expected to change

## Final Validation (All Patches Complete)

**Run complete suite:**
- All unit tests (Level 1)
- All component tests (Level 2)
- All integration tests (Level 3)
- All regression tests (Level 4)

**Generate report:**
```
Validation Summary
==================
Total tests: 50
  Passed: 48
  Failed: 2
  Warnings: 3

Critical tests: 30 / 30 passed ✓

Physics validation: PASS
  - Energy conservation: ✓
  - Static trim: ✓
  - Coriolis properties: ✓
  - Damping dissipation: ✓

Control validation: PASS
  - Speed control: ✓
  - Depth control: ✓
  - Heading control: ✓

Known issues:
  - Test 3.3.2 (Spiral dive): Slight overshoot in depth (within 10%)
  - Test 4.3.1 (Literature comparison): Time constants 15% different (acceptable)
```

---

# ACCEPTANCE CRITERIA

## Minimum Requirements (Must Pass)

**Critical Physics Tests:**
1. ✓ Euler kinematics correct (Test 1.1.1, 1.1.3)
2. ✓ Coriolis skew-symmetric (Test 1.2.1)
3. ✓ Static equilibrium (Test 2.2.1)
4. ✓ Energy conservation when no dissipation (Test 4.1.1)
5. ✓ Energy decreases with damping (Test 4.1.2)

**Critical Control Tests:**
6. ✓ Speed control stable (Test 3.2.1)
7. ✓ Depth control reaches target (Test 3.2.2)
8. ✓ Heading control stable (Test 3.2.3)

**Showstopper Failures:**
- ✗ Any test showing energy increase (non-physical)
- ✗ Instability (divergence) in nominal scenarios
- ✗ Gimbal lock without warning

## Desired (Should Pass)

9. ○ All free decay tests (Test 3.1.x)
10. ○ Turn dynamics realistic (Test 2.3.1)
11. ○ Mission scenarios complete (Test 3.3.x)

## Optional (Nice to Have)

12. ◇ Literature comparison quantitative (Test 4.3.1)
13. ◇ All unit tests pass (some may be very strict)

---

# INSTRUMENTATION & METRICS

## Data Logging Requirements

**For all tests, log:**
```python
# Time series
time: [s]
position_ned: [x, y, z] in meters
orientation_euler: [phi, theta, psi] in radians
velocity_body: [u, v, w] in m/s
angular_velocity_body: [p, q, r] in rad/s
thrust: [N]
fin_deflections: [δ1, δ2, δ3, δ4] in radians

# Derived quantities
energy_kinetic: [J]
energy_potential: [J]
energy_total: [J]
power_thrust: [W]
power_damping: [W]

# Control metrics
command_speed: [m/s]
command_depth: [m]
command_heading: [deg]
error_speed: [m/s]
error_depth: [m]
error_heading: [deg]
```

**Sample rate:** 50 Hz minimum (higher for short tests)

## Computed Metrics

**From time series, extract:**
- Rise time (10%-90%)
- Settling time (±2% band)
- Overshoot (%)
- Steady-state error (mean last 10s)
- RMS tracking error
- Time constants (from exponential fits)
- Natural frequencies (from FFT of oscillations)
- Damping ratios (from overshoot or log decrement)

**Statistical:**
- Mean, std dev, min, max for key variables
- Correlation coefficients (e.g., u vs. thrust)

---

# VISUALIZATION & REPORTING

## Standard Plots (For Each Test)

1. **Trajectory:** 3D path in NED frame
2. **State time histories:** Position, orientation, velocities (6 subplots)
3. **Control time histories:** Commands vs. actuals (4 subplots)
4. **Energy budget:** E_kin, E_pot, E_total vs. time
5. **Phase portraits:** (v vs. v_dot) for damping visualization

## Comparison Plots (Before/After Each Patch)

- Overlay before (red) and after (blue) trajectories
- Difference plots (after - before)
- Metric bar charts (overshoot, settling time, etc.)

## Final Report

**Automated generation:**
```python
python scripts/generate_validation_report.py --output=ValidationReport.pdf
```

**Contents:**
- Executive summary (1 page)
- Test results table (pass/fail for all tests)
- Key metrics comparison (before/after)
- Selected plots (most important 10-15)
- Recommendations and future work

---

# IMPLEMENTATION NOTES

## Test Execution

**Framework:** pytest + custom physics validators

**Directory structure:**
```
tests/
  validation/
    unit/
      test_kinematics.py
      test_coriolis.py
      test_restoring.py
    component/
      test_damping.py
      test_trim.py
      test_turn.py
    integration/
      test_free_decay.py
      test_step_response.py
      test_missions.py
    regression/
      test_energy_budget.py
      test_literature.py
    conftest.py  (fixtures, utilities)
  golden/
    baseline_unfixed/
    baseline_patch1/
    ...
```

**Run subsets:**
```bash
pytest tests/validation/unit/  # Fast (~1 min)
pytest tests/validation/component/  # Medium (~5 min)
pytest tests/validation/integration/  # Slow (~30 min)
pytest tests/validation/ -m critical  # Critical tests only (~10 min)
```

## Continuous Integration

**On each commit/PR:**
- Run critical tests (8 tests, ~2 min)
- Check for physics violations (energy increase, etc.)
- Report pass/fail status

**Nightly:**
- Run full validation suite
- Generate plots
- Email summary report

---

# CONCLUSION

This validation plan provides:
1. **Comprehensive coverage** (unit → component → integration → regression)
2. **Objective criteria** (quantitative thresholds for pass/fail)
3. **Physics-grounded** (tests fundamental laws, not just "looks good")
4. **Repeatable** (automated, version-controlled)
5. **Traceable** (each test references specific audit findings)

**Use this plan to:**
- Validate each patch in `ChangePlan.md`
- Demonstrate correctness to reviewers/advisors
- Build confidence in corrected physics model
- Enable future model refinements with regression protection

**Next steps:**
1. Implement test infrastructure (fixtures, utilities)
2. Write tests for current baseline (capture before state)
3. Apply Patch 1, run tests, document changes
4. Iterate through all patches

---

**End of Validation Plan**

**See also:**
- `Physics-Audit.md` for detailed issue descriptions
- `ChangePlan.md` for correction implementation steps

