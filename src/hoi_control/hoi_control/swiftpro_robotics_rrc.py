"""
swiftpro_robotics_rrc.py
========================
Resolved-Rate Control kinematics library for the uArm Swift Pro.

This is a focused clone of swiftpro_robotics.py retaining ONLY the mathematics
needed for Resolved-Rate Control (RRC):
  • Forward Kinematics  (swiftpro_fk)
  • Geometric Jacobians (3-DOF 3×3, 4-DOF 3×4 position, 4-DOF 4×4 position+yaw)
  • Damped Least-Squares pseudo-inverse  (DLS, weighted_DLS)
  • Velocity scaling  (scale_velocities)
  • SwiftProManipulator4DOF — kinematic state manager for 4 active joints

REMOVED compared to swiftpro_robotics.py:
  • Analytical IK (swiftpro_ik)
  • Characteristic-points helper (swiftpro_points)
  • Task classes (Position3D, YawTask, Configuration3D, JointLimits, Obstacle3D …)
  • Task-priority solver (task_priority_step)
  • MobileManipulator (base + arm combined model)
  • Frame conversion helpers (ned_to_enu, enu_to_ned)

ADDED compared to swiftpro_robotics.py:
  • Joint 4 — revolute end-effector yaw — fully accounted for in FK and Jacobian.

KEY DESIGN DECISION – No Denavit-Hartenberg:
  The uArm Swift Pro uses a parallelogram (closed-chain) linkage.  Standard DH
  convention is invalid.  All kinematics are derived geometrically.

  The parallelogram constraint enforces:
      passive_q4 = -(q2 + q3)    (handled by swiftpro_controller)
  This keeps the wrist link always horizontal, reducing independent position
  DOF to 3: [q1, q2, q3].  Joint4 (controllable) adds EE yaw.

Coordinate convention – arm-local ENU (East-North-Up):
  All FK and Jacobian outputs are in the arm-local ENU frame whose origin is
  the J1 axis (link1 position).
      x_enu = East   (arm reach direction at q1 = 0, sign: negative due to URDF)
      y_enu = North  (arm reach direction at q1 = π/2)
      z_enu = Up     (positive above J1)

Authors: HOI Team  (Haadi / Huy / Phu)
"""

import numpy as np

# ---------------------------------------------------------------------------
# uArm Swift Pro – geometric link parameters (metres)
# ---------------------------------------------------------------------------
# Calibrated against live TF data (ros2 run tf2_ros tf2_echo world_enu link9)
# at q = [π/2, -0.997, 0.05], which gave EE at [0, 0.159, 0.480] world_enu
# and link1 (J1 axis) at [0, -0.051, 0.198] world_enu.
#
# KEY DISCOVERIES vs. the DH table in Schütz & Hepworth (2024):
#   D1: Vertical height from J1 axis to J2/J3 pivot level = 33.3 mm.
#   A1: Horizontal offset J1→J2 is negligible in the FK.
#   L2, L3, L4: confirmed from DH table.

A1 = 0.0      # effective horizontal J1→J2 offset (negligible)
D1 = 0.0333   # vertical height: J1 axis → J2 shoulder pivot (33.3 mm)
L2 = 0.142    # upper arm length  J2 → J3 pivot
L3 = 0.1588   # forearm length    J3 → J4 bracket (link8A)
L4 = 0.0445   # wrist link; always horizontal due to parallelogram constraint

# ---------------------------------------------------------------------------
# Joint 4 — revolute end-effector yaw  (NEW vs. swiftpro_robotics.py)
# ---------------------------------------------------------------------------
# URDF entry:
#   <joint name="${prefix}/joint4" type="revolute">
#       <axis xyz="0 0 1"/>
#       <limit effort="1000.0" lower="-1.571" upper="1.571" velocity="0.2"/>
#       <parent link="${prefix}/link8A"/>
#       <child  link="${prefix}/link8B"/>
#       <origin xyz="0.0565 0 0.0" rpy="0 0 0"/>
#   </joint>
#
# Kinematic interpretation (arm-local ENU):
#   • The joint4 pivot is 56.5 mm ahead of link8A along the horizontal
#     reach direction — the same axis as q1 in the ENU convention.
#   • Its rotation axis [0,0,1] in the URDF aligns with the vertical (ENU z)
#     because the parallelogram keeps link8A always horizontal.
#   • Joint4 therefore adds an independent EE yaw degree of freedom.
#
# Position effect of q4:
#   The EE position (link8B origin = joint4 pivot) is independent of q4 when
#   no tool tip extends beyond the pivot (L_EE_TOOL = 0).  If a tool of
#   length L_EE_TOOL is attached along link8B's x-axis, the EE moves as:
#       Δx = -L_EE_TOOL · cos(q1 + q4)
#       Δy =  L_EE_TOOL · sin(q1 + q4)
#       Δz = 0
#   and the 4th Jacobian column becomes [L_EE_TOOL·sin(q1+q4),
#                                         L_EE_TOOL·cos(q1+q4), 0].
#   Set L_EE_TOOL to the known tool length; leave at 0 if none.

L_J4     = 0.0565   # link8A origin → joint4 pivot (along reach direction, m)
L_EE_TOOL = 0.0     # tool-tip extension from joint4 pivot along link8B x-axis (m)

# Joint angle limits (rad) — from the URDF <limit> tags
# A small inset margin (0.07 rad) is applied in the velocity-clamping code
# of the control node to stop the joint just before the hard limit.
Q1_MIN = -1.571   # joint1 base yaw   lower (−π/2)
Q1_MAX =  1.571   # joint1 base yaw   upper (+π/2)
Q2_MIN = -1.571   # joint2 shoulder   lower (−π/2)
Q2_MAX =  0.05    # joint2 shoulder   upper
Q3_MIN = -1.571   # joint3 elbow      lower (−π/2)
Q3_MAX =  0.05    # joint3 elbow      upper
Q4_MIN = -1.571   # joint4 EE yaw     lower (−π/2)
Q4_MAX =  1.571   # joint4 EE yaw     upper (+π/2)

# ---------------------------------------------------------------------------
# Joint index mapping in /joint_states
# ---------------------------------------------------------------------------
JOINT_NAMES = [
    'turtlebot/swiftpro/joint1',   # index 0 – base yaw   (q1)
    'turtlebot/swiftpro/joint2',   # index 1 – shoulder   (q2)
    'turtlebot/swiftpro/joint3',   # index 2 – elbow      (q3)
]

# 4-DOF name list — same order as the q vector [q1, q2, q3, q4]
JOINT_NAMES_4DOF = JOINT_NAMES + ['turtlebot/swiftpro/joint4']


# ---------------------------------------------------------------------------
# Geometric Forward Kinematics  (3-DOF, unchanged from swiftpro_robotics.py)
# ---------------------------------------------------------------------------

def swiftpro_fk(q):
    """
    Geometric forward kinematics for the uArm Swift Pro.

    OUTPUT FRAME: arm-local ENU, relative to J1 (link1) position.
      x_enu = East   (arm reach at q1=0, NEGATIVE sign – see below)
      y_enu = North  (arm reach at q1=π/2)
      z_enu = Up     (positive above J1)

    Why arm-local ENU (not NED)?
    ──────────────────────────────
    TF measurements confirm:
      • At q1=π/2 the arm reaches in the +y_enu (North) direction.
      • Height is POSITIVE upward (z_enu > 0 when arm is raised).

    FK derivation (URDF-derived, corrected April 2026):
    ─────────────────────────────────────────────────────
    Horizontal reach from J1 vertical axis:
        r = 0.0698 − L2·sin(q2) + L3·cos(q3)

    Height above J1 (ENU):
        z = 0.0334 + L2·cos(q2) + L3·sin(q3)

    3-D position:
        x_enu = −r · cos(q1)   ← negative sign confirmed against TF
        y_enu =  r · sin(q1)
        z_enu = z

    Verified against TF:
        q=(π/2, 0.050, 0.050): r=192mm, z=283mm ✓
        q=(π/2, −1.571, 0.050): r=341mm ✓

    Arguments
    ---------
    q : array-like, shape (3,) or (4,)
        [q1, q2, q3] or [q1, q2, q3, q4].
        Only q1, q2, q3 affect the EE position; q4 adds EE yaw (orientation).

    Returns
    -------
    p : np.ndarray, shape (3,)  – [x, y, z] in arm-local ENU (metres)
    """
    q1, q2, q3 = float(q[0]), float(q[1]), float(q[2])

    r     = 0.0698 - L2 * np.sin(q2) + L3 * np.cos(q3)
    z_enu = 0.0334 + L2 * np.cos(q2) + L3 * np.sin(q3)
    x_enu = -r * np.cos(q1)
    y_enu =  r * np.sin(q1)

    return np.array([x_enu, y_enu, z_enu])


# ---------------------------------------------------------------------------
# 3-DOF Geometric Jacobian (position only, 3×3)
# ---------------------------------------------------------------------------

def swiftpro_jacobian(q):
    """
    Geometric (analytic) Jacobian mapping [dq1, dq2, dq3] → EE linear velocity.
    OUTPUT FRAME: arm-local ENU.

    Derivation (partial derivatives of swiftpro_fk):
    ─────────────────────────────────────────────────
        r   = 0.0698 − L2·sin(q2) + L3·cos(q3)
        x   = −r·cos(q1),   y = r·sin(q1),   z = ...

    ∂r/∂q2 = −L2·cos(q2)       ∂r/∂q3 = −L3·sin(q3)
    ∂z/∂q2 = −L2·sin(q2)       ∂z/∂q3 =  L3·cos(q3)

    ∂x/∂q1 = r·sin(q1)         (from d(−r·cos)/dq1)
    ∂y/∂q1 = r·cos(q1)         (from d( r·sin)/dq1)
    ∂x/∂q2 =  L2·cos(q2)·cos(q1)
    ∂y/∂q2 = −L2·cos(q2)·sin(q1)   ← sign from x=−r·cos + chain rule
    ∂x/∂q3 =  L3·sin(q3)·cos(q1)
    ∂y/∂q3 = −L3·sin(q3)·sin(q1)

    Arguments
    ---------
    q : array-like, shape (3,) or (4,) – only first 3 elements used.

    Returns
    -------
    J : np.ndarray, shape (3, 3)
        Rows → [dx_enu, dy_enu, dz_enu];  Columns → [q1, q2, q3].
    """
    q1, q2, q3 = float(q[0]), float(q[1]), float(q[2])

    s1, c1 = np.sin(q1), np.cos(q1)
    s2, c2 = np.sin(q2), np.cos(q2)
    s3, c3 = np.sin(q3), np.cos(q3)

    r      = 0.0698 - L2 * s2 + L3 * c3

    dr_dq2 = -L2 * c2
    dr_dq3 = -L3 * s3
    dz_dq2 = -L2 * s2
    dz_dq3 =  L3 * c3

    J = np.array([
        # ── q1 ──────    ── q2 ───────────────    ── q3 ────────────
        [  r * s1,        -dr_dq2 * c1,            -dr_dq3 * c1  ],   # dx_enu
        [  r * c1,         dr_dq2 * s1,             dr_dq3 * s1  ],   # dy_enu
        [  0.0,            dz_dq2,                   dz_dq3      ],   # dz_enu
    ])
    return J


# ---------------------------------------------------------------------------
# 4-DOF Jacobians  (NEW — include joint4)
# ---------------------------------------------------------------------------

def swiftpro_jacobian_pos4(q):
    """
    4-DOF position Jacobian mapping [dq1, dq2, dq3, dq4] → EE linear velocity.
    Shape: (3, 4).

    Joint4 column:
    ──────────────
    Joint4 is a revolute joint whose axis passes through the EE (or near it).
    When no tool tip extends beyond the joint4 pivot (L_EE_TOOL = 0), the
    moment arm to the EE is zero and ∂p/∂q4 = [0, 0, 0].

    When a tool of length L_EE_TOOL is attached along link8B's x-axis:
        ∂x/∂q4 = L_EE_TOOL · sin(q1 + q4)
        ∂y/∂q4 = L_EE_TOOL · cos(q1 + q4)
        ∂z/∂q4 = 0
    (Adjust L_EE_TOOL at module level to match your physical tool.)

    Arguments
    ---------
    q : array-like, shape (4,) – [q1, q2, q3, q4]

    Returns
    -------
    J : np.ndarray, shape (3, 4)
    """
    q1, q4 = float(q[0]), float(q[3])
    J3 = swiftpro_jacobian(q)   # (3, 3)

    # 4th column: tool-tip position sensitivity to q4
    angle_j4 = q1 + q4   # direction of link8B in the horizontal plane
    col4 = np.array([
        L_EE_TOOL * np.sin(angle_j4),
        L_EE_TOOL * np.cos(angle_j4),
        0.0
    ])

    return np.hstack([J3, col4.reshape(3, 1)])   # (3, 4)


def swiftpro_jacobian_full4(q):
    """
    4-DOF full Jacobian mapping [dq1, dq2, dq3, dq4] → [dx, dy, dz, d(yaw)].
    Shape: (4, 4).  Use this for combined position + EE yaw tasks.

    Yaw row derivation:
    ───────────────────
    EE yaw = q1 + q4   (joint4 rotates the EE tip about the same vertical axis
                         as q1; parallelogram constraint keeps pitch/roll fixed).

        ∂yaw/∂q1 = 1,  ∂yaw/∂q2 = 0,  ∂yaw/∂q3 = 0,  ∂yaw/∂q4 = 1

    The error vector for this Jacobian is (4×1): [ex, ey, ez, e_yaw].
    Wrap e_yaw to [−π, π] before passing to the RRC law to avoid spinning.

    Arguments
    ---------
    q : array-like, shape (4,)

    Returns
    -------
    J : np.ndarray, shape (4, 4)
    """
    J_pos4 = swiftpro_jacobian_pos4(q)   # (3, 4)
    yaw_row = np.array([[1.0, 0.0, 0.0, 1.0]])   # d(yaw)/d[q1,q2,q3,q4]
    return np.vstack([J_pos4, yaw_row])           # (4, 4)


# ---------------------------------------------------------------------------
# Damped Least-Squares pseudo-inverse  (unchanged from swiftpro_robotics.py)
# ---------------------------------------------------------------------------

def DLS(A, damping):
    """
    Damped Least-Squares pseudo-inverse.
        A_dls = Aᵀ (A Aᵀ + λ²I)⁻¹

    Preferred over the plain pseudo-inverse near singular configurations
    because it trades accuracy for numerical stability.  The damping factor
    λ controls the trade-off: larger λ → smoother motion, larger steady-state
    error; λ = 0 → plain pseudo-inverse.
    """
    lam2 = damping ** 2
    m    = A.shape[0]
    return A.T @ np.linalg.inv(A @ A.T + lam2 * np.eye(m))


def weighted_DLS(A, damping, W):
    """
    Weighted Damped Least-Squares:
        A_wdls = W⁻¹ Aᵀ (A W⁻¹ Aᵀ + λ²I)⁻¹

    W is a positive-definite weight matrix (e.g. diagonal with joint velocity
    limits on the diagonal) that penalises fast joints more heavily.
    """
    lam2  = damping ** 2
    m     = A.shape[0]
    W_inv = np.linalg.inv(W)
    return W_inv @ A.T @ np.linalg.inv(A @ W_inv @ A.T + lam2 * np.eye(m))


def scale_velocities(zeta, max_vel):
    """
    Scale the joint velocity vector uniformly so that no element exceeds
    max_vel in absolute value.  Preserves the direction of motion.
    """
    s = np.max(np.abs(zeta)) / max_vel
    if s > 1.0:
        zeta = zeta / s
    return zeta


# ===========================================================================
#  SwiftProManipulator4DOF  –  4-DOF kinematic state (q1, q2, q3, q4)
# ===========================================================================

class SwiftProManipulator4DOF:
    """
    Kinematic state manager for the uArm Swift Pro with joint4.

    Wraps the four active joint angles [q1, q2, q3, q4] and exposes FK /
    Jacobian queries needed by the RRC control loop.

    Unlike the 3-DOF SwiftProManipulator in swiftpro_robotics.py, this class:
      • Tracks q4 (end-effector yaw joint)
      • Provides both position-only (3×4) and full (4×4) Jacobians
      • Reports EE yaw as q1 + q4 (both joints rotate about the vertical axis)

    Usage (inside a ROS node):
    --------------------------
        arm = SwiftProManipulator4DOF()
        arm.update_from_joint_states(msg.name, msg.position)
        J    = arm.getEEJacobianPos()    # (3, 4) position Jacobian
        J4   = arm.getEEJacobianFull()   # (4, 4) position + yaw Jacobian
        p    = arm.getEEPosition()       # (3,) position
        yaw  = arm.getEEYaw()            # scalar EE yaw = q1 + q4
        arm.integrate(dq, dt)            # forward-Euler joint update
    """

    DOF = 4   # q1, q2, q3, q4 (all active)

    def __init__(self, q0=None):
        """
        q0 : initial joint angles [q1, q2, q3, q4] (rad).
             Defaults to all zeros.
        """
        self.q = np.zeros(4) if q0 is None else np.array(q0, dtype=float)

    # ------------------------------------------------------------------ #
    #  State update                                                        #
    # ------------------------------------------------------------------ #

    def update_from_joint_states(self, names, positions):
        """
        Parse a sensor_msgs/JointState message and extract q1–q4.

        Joint4 may not be published if the simulator does not include it in
        the active controller; in that case q4 stays at its previous value.

        Arguments
        ---------
        names     : list[str]   – JointState.name
        positions : list[float] – JointState.position  (same order)
        """
        pos_map = dict(zip(names, positions))
        for i, jn in enumerate(JOINT_NAMES_4DOF):
            if jn in pos_map:
                self.q[i] = pos_map[jn]

    def integrate(self, dq, dt):
        """Forward-Euler integration: q ← q + dq · dt."""
        self.q = self.q + np.array(dq, dtype=float).flatten()[:4] * dt

    # ------------------------------------------------------------------ #
    #  Kinematics queries                                                  #
    # ------------------------------------------------------------------ #

    def getDOF(self):
        return self.DOF

    def getJointAngles(self):
        """Returns a copy of [q1, q2, q3, q4]."""
        return self.q.copy()

    def getJointPos(self, idx):
        """Return scalar angle of joint idx (0-based)."""
        return float(self.q[idx])

    def getEEPosition(self):
        """
        3-D EE position in arm-local ENU (metres), relative to J1.
        Only q1, q2, q3 determine position; q4 is EE yaw (orientation only,
        assuming L_EE_TOOL = 0).
        """
        return swiftpro_fk(self.q)

    def getEEYaw(self):
        """
        Scalar EE yaw angle (radians) in arm-local ENU.
        yaw = q1 + q4  — both joints rotate about the vertical axis.
        """
        return float(self.q[0]) + float(self.q[3])

    def getEEJacobianPos(self):
        """
        3×4 position Jacobian  (maps dq → linear EE velocity in arm-local ENU).
        4th column = [0, 0, 0] when L_EE_TOOL = 0 (q4 is a pure orientation DOF).
        """
        return swiftpro_jacobian_pos4(self.q)

    def getEEJacobianFull(self):
        """
        4×4 Jacobian  (maps dq → [dx, dy, dz, d(yaw)] in arm-local ENU).
        Use this for combined position + EE yaw tasks.
        """
        return swiftpro_jacobian_full4(self.q)
