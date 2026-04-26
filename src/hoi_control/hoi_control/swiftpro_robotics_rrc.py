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
  convention is invalid.  All kinematics are derived geometrically by tracing
  the URDF joint chain link by link.

  The parallelogram constraint enforces:
      passive_q4 = -(q2 + q3)    (handled by swiftpro_controller)
  This keeps the wrist link always horizontal, reducing independent position
  DOF to 3: [q1, q2, q3].  Joint4 (controllable) adds EE yaw.

OUTPUT FRAME — world_enu-aligned, relative to J1:
  The FK outputs a position vector whose axes are aligned with world_enu
  (x=East, y=North, z=Up), but whose origin is J1 (link1's position in
  the world).  To get the absolute world_enu position of the EE, add J1's
  world_enu position (obtained from TF) to the FK output.

  This is NOT the same as link1's local frame.  link1's local frame has
  z pointing downward (NED convention from Stonefish).  The FK outputs
  in world_enu-aligned axes despite using link1 as origin, because the
  geometric derivation was done in NED and then converted to ENU at the end.

Authors: HOI Team  (Haadi / Huy / Phu)
"""

import numpy as np

# ---------------------------------------------------------------------------
# uArm Swift Pro – geometric link parameters (metres)
# ---------------------------------------------------------------------------
# These constants were derived by tracing the URDF joint chain geometrically
# (no DH convention — invalid for closed-chain parallelogram linkages).
#
# The kinematic chain traced is:
#   link1 → joint2 → link2 → passive_joint1 → link3
#         → passive_joint7 → link8A → joint4 → link8B → end_effector
#
# L2 and L3 come directly from URDF joint origins:
#   passive_joint1: xyz="0 0 -0.142"  → L2 = 0.142 m (upper arm)
#   passive_joint7: xyz="0.1587 0 0"  → L3 = 0.1588 m (forearm, rounded)
#
# The base constants in swiftpro_fk (0.0127 for reach, -0.0382 for height)
# are NOT pure URDF values — they were empirically recalibrated by comparing
# FK output against TF ground truth at known joint configurations after
# switching the EE frame from link9 to end_effector (link8B + 0.0722m up).
# See swiftpro_fk docstring for the full derivation and calibration history.

A1 = 0.0      # effective horizontal J1→J2 offset (negligible, from URDF joint2 x=0.0133)
D1 = 0.0333   # vertical height: J1 axis → J2 shoulder pivot (33.3 mm, from URDF joint2 z=-0.1056)
L2 = 0.142    # upper arm length  J2 → J3 pivot (from URDF passive_joint1 z=-0.142)
L3 = 0.1588   # forearm length    J3 → link8A  (from URDF passive_joint7 x=0.1587, rounded)
L4 = 0.0445   # wrist link; always horizontal due to parallelogram constraint

# ---------------------------------------------------------------------------
# Joint 4 — revolute end-effector yaw
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
# L_J4 = 0.0565 m is the horizontal distance from link8A's origin to link8B's
# origin (the joint4 pivot).  The parallelogram keeps link8A always horizontal,
# so this offset is always along the arm's reach direction in world_enu,
# regardless of q2 or q3.  It is added to the horizontal reach r in swiftpro_fk
# as a constant, and its contribution to the Jacobian appears only in the q1
# column (since q1 sweeps the full reach including this offset).
#
# L_EE_TOOL: if a physical tool of known length is attached along link8B's
# x-axis, set this to its length.  It extends the reach and adds a q4-dependent
# position term.  Leave at 0 if no tool is attached (current setup).

L_J4      = 0.0565   # link8A origin → joint4 pivot / link8B origin (m)
L_EE_TOOL = 0.0      # tool-tip extension beyond link8B origin along link8B x-axis (m)

# Joint angle limits (rad) — copied directly from the URDF <limit> tags.
# A small inset margin (LIMIT_MARGIN = 0.07 rad) is applied in the control node
# to stop joints slightly before the hard limit, giving the controller time
# to decelerate.
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
# Geometric Forward Kinematics
# ---------------------------------------------------------------------------

def swiftpro_fk(q):
    """
    Geometric forward kinematics for the uArm Swift Pro.

    Returns the 3-D position of the end_effector frame origin relative to J1
    (link1's world position), expressed in world_enu-aligned axes.

    OUTPUT FRAME
    ────────────
    The output is a displacement vector in world_enu axes (x=East, y=North,
    z=Up), measured from J1's world position.  To get the absolute world_enu
    position of the EE, add J1's world_enu position from TF:

        ee_world_enu = swiftpro_fk(q) + j1_world_enu

    This frame is world_enu-ALIGNED but J1-CENTRED.  It is NOT the same as
    link1's local frame, which has z pointing downward (Stonefish NED).

    KINEMATIC DERIVATION (URDF-based, no DH)
    ─────────────────────────────────────────
    Step 1 — Work in NED, with q1=0, in the xz plane.
      The URDF uses NED (x=North, y=East, z=Down).  We fix q1=0 and compute
      the 2-D geometry in the vertical plane first, then rotate by q1 at the end.

    Step 2 — Horizontal reach r (distance from J1 yaw axis to EE, in the plane).
      Traced from URDF joint origins:
        joint2 origin:       x=+0.0133 m  (shoulder is 13.3mm forward of J1)
        passive_joint1:      z=-0.142 m   (upper arm, length L2=0.142m, rotated by q2)
        passive_joint7:      x=+0.1587 m  (forearm, length L3=0.1588m, rotated by q3)
        joint4 (to link8B):  x=+0.0565 m  (L_J4, always horizontal, no rotation)

      The parallelogram constraint means link3's frame always has the same
      orientation as link1's frame, regardless of q2.  So the forearm direction
      in the world depends only on q3, not on q2+q3 as in a normal serial arm.

      After rotating each segment and summing:
        r = 0.0127 - L2·sin(q2) + L3·cos(q3)
        r_ee = r + L_J4   ← extends from link8A to link8B (end_effector pivot)

      The base constant 0.0127 was empirically recalibrated (URDF-derived value
      was 0.0133 for joint2 x-offset alone, without passive_joint8 which is no
      longer in the EE chain).

    Step 3 — Height above J1 (z in NED, then converted to ENU).
      joint2 origin:       z=-0.1056 m  (shoulder is 105.6mm above J1, since NED z<0 = up)
      passive_joint1:      z=-0.142·cos(q2)  (upper arm vertical component)
      passive_joint7:      no z contribution at q3=0 (forearm is horizontal)
                           z contribution = -L3·sin(q3) at angle q3
      end_effector:        +0.0722 m above link8B (static transform in launch file)

      Collecting and converting NED → ENU (z_enu = -z_ned):
        z_enu = -0.0382 + L2·cos(q2) + L3·sin(q3) + 0.0722

      The base constant -0.0382 was empirically recalibrated (the URDF-derived
      constant was +0.1056 for the shoulder height, but after switching EE target
      from link9 to end_effector and accounting for the static transform offset,
      the value was tuned against live TF data to achieve FK_vs_TF < 1mm).

    Step 4 — Apply q1 rotation in the horizontal plane.
      r is the scalar reach of the arm.  q1 rotates that reach around the
      vertical (J1 yaw) axis.  The 3-D position is:
        x_enu = -r_ee · cos(q1_eff)
        y_enu =  r_ee · sin(q1_eff)
      where q1_eff = q1 - π/2 (explained in detail below).

    WHY q1_eff = q1 - π/2  (the π/2 phase shift)
    ──────────────────────────────────────────────
    The formula x = -r·cos(q1), y = r·sin(q1) was originally derived assuming
    that at q1=0 the arm points in the world_enu -x direction (West).  This
    assumption was embedded in the choice of negative cosine for x and positive
    sine for y.

    However, in this simulator the arm physically points in the world_enu -y
    direction (South) when q1=0.  This was confirmed directly from TF data:

        At q=[0, 0, 0, 0], TF reports:
            EE relative to J1 = [0.000, -0.228, 0.176]
        The arm displacement is purely in the -y direction.  No x component.

    The reason is the scenario file (turtlebot_basic.scn):
        <arg name="start_yaw" value="${pi/2.0}"/>
        <arg name="arm_yaw"   value="${pi/2.0}"/>
    The robot body is placed at yaw=π/2 in the world, and the arm mount is
    additionally rotated π/2 on the robot body.  The combined effect is that
    joint1's zero position corresponds to the arm pointing South (-y) in
    world_enu, not West (-x).

    The formula without correction at q1=0:
        cos(0) = 1,  sin(0) = 0
        x = -r * 1 = -r   (predicts West)   ✗
        y =  r * 0 =  0                      ✗

    The formula with q1_eff = q1 - π/2 at q1=0:
        q1_eff = -π/2
        cos(-π/2) = 0,  sin(-π/2) = -1
        x = -r * 0  =  0                     ✓ matches TF
        y =  r * (-1) = -r                   ✓ matches TF

    The shift does NOT add any physics — it only corrects the mismatch between
    what the formula assumed about q1=0 (West) and what the simulator actually
    does (South).  The shift is specific to this scenario's robot placement.
    If the robot were placed at a different world yaw, the shift value would
    change accordingly.

    NUMERICAL VERIFICATION (at q=[0, 0, 0, 0])
    ────────────────────────────────────────────
    TF measured:
        EE relative to J1 = [0.000, -0.228, 0.176]   (world_enu)

    FK with shift (q1_eff = -π/2):
        r    = 0.0127 + L3 = 0.0127 + 0.1588 = 0.1715
        r_ee = 0.1715 + 0.0565 = 0.2280
        x    = -0.2280 * cos(-π/2) = 0.000   ✓
        y    =  0.2280 * sin(-π/2) = -0.228  ✓
        z    = -0.0382 + L2 + 0.0722 = -0.0382 + 0.142 + 0.0722 = 0.176  ✓

    FK_vs_TF = 0.6 mm  (verified in live simulation)

    Arguments
    ---------
    q : array-like, shape (3,) or (4,)
        [q1, q2, q3] or [q1, q2, q3, q4].
        Only q1, q2, q3 affect EE position; q4 adds EE yaw (orientation only,
        assuming L_EE_TOOL = 0).

    Returns
    -------
    p : np.ndarray, shape (3,)
        [x, y, z] displacement from J1 in world_enu-aligned axes (metres).
        Add J1's world_enu position from TF to get absolute world coordinates.
    """
    q1, q2, q3 = float(q[0]), float(q[1]), float(q[2])

    # ── q1 phase correction ───────────────────────────────────────────────
    # The formula x=-r·cos(q1), y=r·sin(q1) assumes q1=0 means arm points West
    # (-x in world_enu).  In this simulator, q1=0 means arm points South (-y).
    # Subtracting π/2 bridges that 90° gap.  See docstring for full explanation.
    q1_eff = q1 - np.pi / 2

    # ── Horizontal reach from J1 yaw axis to link8A ───────────────────────
    # r = base_offset - L2·sin(q2) + L3·cos(q3)
    #
    # -L2·sin(q2): at q2=0 the upper arm points straight up (no horizontal
    #   reach contribution).  As q2 goes negative (arm tilts forward), sin(q2)
    #   becomes negative, making -L2·sin(q2) positive — arm reaches forward.
    #
    # +L3·cos(q3): at q3=0 the forearm is horizontal (full reach contribution).
    #   The parallelogram keeps the forearm's world orientation = q3 only
    #   (not q2+q3), so cos(q3) gives the horizontal projection directly.
    r    = 0.0127 - L2 * np.sin(q2) + L3 * np.cos(q3)

    # ── Height above J1 in ENU ────────────────────────────────────────────
    # z_enu = base_height + L2·cos(q2) + L3·sin(q3)
    #
    # +L2·cos(q2): at q2=0 the upper arm is vertical — full height contribution.
    #   As q2 tilts toward horizontal, cos(q2) → 0.
    #
    # +L3·sin(q3): at q3=0 the forearm is horizontal — no height contribution.
    #   As q3 goes positive (forearm tilts up), sin(q3) > 0 — adds height.
    z_enu = -0.0382 + L2 * np.cos(q2) + L3 * np.sin(q3)

    # ── Extend to end_effector ────────────────────────────────────────────
    # L_J4 = 0.0565 m: horizontal distance from link8A to link8B (joint4 pivot).
    # This offset is always along the arm's reach direction because the
    # parallelogram keeps link8A horizontal regardless of q2, q3.
    r_ee = r + L_J4

    # 0.0722 m: vertical offset from link8B to end_effector, defined by the
    # static_transform_publisher in the launch file (xyz="0 0 0.0722").
    # This is always vertical (world_enu z) because the parallelogram keeps
    # link8B level at all configurations.
    z_ee = z_enu + 0.0722

    # ── Apply q1 rotation to get world_enu x and y ────────────────────────
    # The arm sweeps a horizontal circle of radius r_ee as q1 changes.
    # The negative sign on x comes from the original NED derivation, confirmed
    # empirically against TF data.
    x_enu = -r_ee * np.cos(q1_eff)
    y_enu =  r_ee * np.sin(q1_eff)

    return np.array([x_enu, y_enu, z_ee])


# ---------------------------------------------------------------------------
# 3-DOF Geometric Jacobian (position only, 3×3)
# ---------------------------------------------------------------------------

def swiftpro_jacobian(q):
    """
    Geometric Jacobian mapping joint velocities [dq1, dq2, dq3] → EE linear
    velocity [dx, dy, dz] in world_enu-aligned axes relative to J1.

    This is the partial derivative of swiftpro_fk with respect to each joint
    angle.  It answers: if joint i moves at 1 rad/s, how fast does the EE move
    in x, y, z?  The RRC control law uses this to invert the mapping and compute
    what joint velocities produce a desired EE velocity toward the target.

    DERIVATION (partial derivatives of swiftpro_fk)
    ────────────────────────────────────────────────
    From the FK:
        r     = 0.0127 - L2·sin(q2) + L3·cos(q3)
        r_ee  = r + L_J4
        x     = -r_ee · cos(q1_eff),   q1_eff = q1 - π/2
        y     =  r_ee · sin(q1_eff)
        z     = -0.0382 + L2·cos(q2) + L3·sin(q3) + 0.0722

    Partial derivatives:

    ∂r/∂q2 = -L2·cos(q2)       (dr_dq2)
    ∂r/∂q3 = -L3·sin(q3)       (dr_dq3)
    ∂z/∂q2 = -L2·sin(q2)       (dz_dq2)
    ∂z/∂q3 =  L3·cos(q3)       (dz_dq3)

    For x = -r_ee·cos(q1_eff), using the chain rule:
      ∂x/∂q1 = -r_ee · (-sin(q1_eff)) · 1 = r_ee·sin(q1_eff)  [since dq1_eff/dq1=1]
      ∂x/∂q2 = -cos(q1_eff) · dr_dq2      = L2·cos(q2)·cos(q1_eff)
      ∂x/∂q3 = -cos(q1_eff) · dr_dq3      = L3·sin(q3)·cos(q1_eff)

    For y = r_ee·sin(q1_eff):
      ∂y/∂q1 = r_ee·cos(q1_eff)
      ∂y/∂q2 = sin(q1_eff) · dr_dq2       = -L2·cos(q2)·sin(q1_eff)
      ∂y/∂q3 = sin(q1_eff) · dr_dq3       = L3·sin(q3)·sin(q1_eff)

    NOTE on q1_eff = q1 - π/2:
    The same π/2 shift applied in swiftpro_fk must be applied here.  The
    Jacobian is the derivative of FK, so it must use the same effective angle.
    s1 = sin(q1_eff), c1 = cos(q1_eff) throughout.

    NOTE on r vs r_ee in the q1 column:
    The q1 column uses r_ee (= r + L_J4) because q1 sweeps the TOTAL horizontal
    reach including the joint4 offset.  The q2 and q3 columns use dr_dq2 and
    dr_dq3, which are derivatives of r with respect to q2/q3 — L_J4 is a
    constant and drops out of those partial derivatives.

    Arguments
    ---------
    q : array-like, shape (3,) or (4,) – only first 3 elements used.

    Returns
    -------
    J : np.ndarray, shape (3, 3)
        Rows → [dx, dy, dz] in world_enu-aligned axes.
        Columns → [dq1, dq2, dq3].
    """
    q1, q2, q3 = float(q[0]), float(q[1]), float(q[2])

    # Same π/2 shift as FK — Jacobian must be consistent with FK
    q1_eff = q1 - np.pi / 2
    s1, c1 = np.sin(q1_eff), np.cos(q1_eff)

    s2, c2 = np.sin(q2), np.cos(q2)
    s3, c3 = np.sin(q3), np.cos(q3)

    r      = 0.0127 - L2 * s2 + L3 * c3   # reach to link8A (note: uses original r constant)
    r_ee   = r + L_J4                       # total reach including joint4 offset

    dr_dq2 = -L2 * c2   # ∂r/∂q2
    dr_dq3 = -L3 * s3   # ∂r/∂q3
    dz_dq2 = -L2 * s2   # ∂z/∂q2
    dz_dq3 =  L3 * c3   # ∂z/∂q3

    J = np.array([
        # ── q1 column ──────    ── q2 column ─────────    ── q3 column ──────
        [  r_ee * s1,            -dr_dq2 * c1,             -dr_dq3 * c1  ],  # ∂x/∂qi
        [  r_ee * c1,             dr_dq2 * s1,              dr_dq3 * s1  ],  # ∂y/∂qi
        [  0.0,                   dz_dq2,                    dz_dq3      ],  # ∂z/∂qi
    ])
    return J


# ---------------------------------------------------------------------------
# 4-DOF Jacobians  (include joint4 EE yaw)
# ---------------------------------------------------------------------------

def swiftpro_jacobian_pos4(q):
    """
    4-DOF position Jacobian mapping [dq1, dq2, dq3, dq4] → EE linear velocity.
    Shape: (3, 4).

    The first three columns are identical to swiftpro_jacobian (3×3).
    The 4th column is the partial derivative of EE position with respect to q4.

    Joint4 column derivation:
    ─────────────────────────
    Joint4 is a revolute joint whose axis is vertical (z-up in world_enu) and
    whose pivot passes through the EE origin (link8B).  When no tool extends
    beyond the pivot (L_EE_TOOL = 0), rotating q4 does not move the EE position
    — it only rotates the EE frame.  Therefore ∂p/∂q4 = [0, 0, 0].

    If L_EE_TOOL > 0 (a physical tool attached along link8B's x-axis), the EE
    tip moves as q4 changes:
        ∂x/∂q4 = L_EE_TOOL · sin(q1_eff + q4)
        ∂y/∂q4 = L_EE_TOOL · cos(q1_eff + q4)
        ∂z/∂q4 = 0
    Set L_EE_TOOL to the tool length if needed; currently 0.

    Arguments
    ---------
    q : array-like, shape (4,) – [q1, q2, q3, q4]

    Returns
    -------
    J : np.ndarray, shape (3, 4)
    """
    q1, q4 = float(q[0]), float(q[3])
    J3 = swiftpro_jacobian(q)   # (3, 3) — first three columns

    # 4th column: EE position sensitivity to q4
    # With L_EE_TOOL=0 this is [0,0,0]; non-zero only if a tool is mounted.
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
    Shape: (4, 4).  Use this for combined position + EE yaw control.

    The top 3 rows are swiftpro_jacobian_pos4 (position).
    The bottom row maps joint velocities to EE yaw rate.

    Yaw row derivation:
    ───────────────────
    EE yaw in world_enu = q1 + q4.
    Both q1 (base yaw) and q4 (EE yaw joint) rotate the EE about the same
    vertical axis.  The parallelogram constraint keeps pitch and roll fixed.

        ∂yaw/∂q1 = 1,  ∂yaw/∂q2 = 0,  ∂yaw/∂q3 = 0,  ∂yaw/∂q4 = 1

    When using this Jacobian, the error vector must be 4×1: [ex, ey, ez, e_yaw].
    Always wrap e_yaw to [−π, π] before use to avoid the controller spinning
    the long way around.

    Arguments
    ---------
    q : array-like, shape (4,)

    Returns
    -------
    J : np.ndarray, shape (4, 4)
    """
    J_pos4  = swiftpro_jacobian_pos4(q)              # (3, 4)
    yaw_row = np.array([[1.0, 0.0, 0.0, 1.0]])       # ∂yaw/∂[q1,q2,q3,q4]
    return np.vstack([J_pos4, yaw_row])               # (4, 4)


# ---------------------------------------------------------------------------
# Damped Least-Squares pseudo-inverse
# ---------------------------------------------------------------------------

def DLS(A, damping):
    """
    Damped Least-Squares pseudo-inverse.
        A_dls = Aᵀ (A Aᵀ + λ²I)⁻¹

    Preferred over the plain Moore-Penrose pseudo-inverse near singular
    configurations because it trades a small amount of accuracy for numerical
    stability.  Near singularities, A·Aᵀ becomes ill-conditioned and its
    plain inverse explodes — adding λ²·I ensures the matrix is always
    invertible.

    Trade-off:
      larger λ → smoother motion, larger steady-state position error.
      smaller λ → more accurate, approaches plain pseudo-inverse behaviour.
      λ = 0     → exactly equivalent to plain pseudo-inverse.
    """
    lam2 = damping ** 2
    m    = A.shape[0]
    return A.T @ np.linalg.inv(A @ A.T + lam2 * np.eye(m))


def weighted_DLS(A, damping, W):
    """
    Weighted Damped Least-Squares pseudo-inverse.
        A_wdls = W⁻¹ Aᵀ (A W⁻¹ Aᵀ + λ²I)⁻¹

    W is a positive-definite weight matrix that penalises certain joints more
    heavily.  For example, a diagonal W with joint velocity limits on the
    diagonal prevents fast joints from dominating the solution.
    """
    lam2  = damping ** 2
    m     = A.shape[0]
    W_inv = np.linalg.inv(W)
    return W_inv @ A.T @ np.linalg.inv(A @ W_inv @ A.T + lam2 * np.eye(m))


def scale_velocities(zeta, max_vel):
    """
    Scale the joint velocity vector uniformly so that no element exceeds
    max_vel in absolute value.  Preserves the direction of motion — all
    joints slow down by the same factor so the arm still moves toward the
    target, just slower.

    If the maximum absolute value in zeta is already ≤ max_vel, no scaling
    is applied and zeta is returned unchanged.
    """
    s = np.max(np.abs(zeta)) / max_vel
    if s > 1.0:
        zeta = zeta / s
    return zeta


# ===========================================================================
#  SwiftProManipulator4DOF  –  4-DOF kinematic state manager
# ===========================================================================

class SwiftProManipulator4DOF:
    """
    Kinematic state manager for the uArm Swift Pro with 4 active joints.

    Wraps the four active joint angles [q1, q2, q3, q4] and exposes FK and
    Jacobian queries needed by the RRC control loop.  Updated each control
    tick from the /joint_states topic via update_from_joint_states().

    Joint roles:
      q1 — base yaw:     rotates the whole arm around the vertical axis
      q2 — shoulder:     tilts the upper arm up/down
      q3 — elbow:        tilts the forearm (but due to parallelogram, this is
                         independent of q2 in world_enu — see FK docstring)
      q4 — EE yaw:       rotates the end-effector about the vertical axis

    Usage inside a ROS node:
        arm = SwiftProManipulator4DOF()
        arm.update_from_joint_states(msg.name, msg.position)
        p   = arm.getEEPosition()       # (3,) EE position relative to J1
        J   = arm.getEEJacobianPos()   # (3, 4) position Jacobian
        J4  = arm.getEEJacobianFull()  # (4, 4) position + yaw Jacobian
        yaw = arm.getEEYaw()           # scalar EE yaw = q1 + q4
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

        Joints are matched by name (not by index) so the ordering in the
        JointState message does not matter.  If joint4 is not present in the
        message (e.g. the simulator does not publish it), q4 retains its
        previous value.

        Arguments
        ---------
        names     : list[str]   – JointState.name
        positions : list[float] – JointState.position  (same length as names)
        """
        pos_map = dict(zip(names, positions))
        for i, jn in enumerate(JOINT_NAMES_4DOF):
            if jn in pos_map:
                self.q[i] = pos_map[jn]

    def integrate(self, dq, dt):
        """
        Forward-Euler integration: q ← q + dq · dt.

        Used when the control node maintains its own internal joint angle
        estimate (e.g. when /joint_states is unavailable or delayed).
        In normal operation, update_from_joint_states() is preferred because
        it uses the simulator's ground-truth joint positions.
        """
        self.q = self.q + np.array(dq, dtype=float).flatten()[:4] * dt

    # ------------------------------------------------------------------ #
    #  Kinematics queries                                                  #
    # ------------------------------------------------------------------ #

    def getDOF(self):
        return self.DOF

    def getJointAngles(self):
        """Returns a copy of [q1, q2, q3, q4] in radians."""
        return self.q.copy()

    def getJointPos(self, idx):
        """Return scalar angle of joint idx (0-based) in radians."""
        return float(self.q[idx])

    def getEEPosition(self):
        """
        3-D EE position in world_enu-aligned axes, relative to J1 (metres).
        Only q1, q2, q3 affect position; q4 is pure orientation (yaw only).
        Add J1's world_enu position from TF to get absolute world coordinates.
        """
        return swiftpro_fk(self.q)

    def getEEYaw(self):
        """
        Scalar EE yaw angle in world_enu (radians).
        yaw = q1 + q4: both joints rotate the EE about the vertical axis.
        The parallelogram constraint keeps pitch and roll fixed at all times.
        """
        return float(self.q[0]) + float(self.q[3])

    def getEEJacobianPos(self):
        """
        3×4 position Jacobian mapping dq → linear EE velocity in
        world_enu-aligned axes.  4th column = [0,0,0] when L_EE_TOOL=0
        (q4 is a pure orientation DOF with no positional effect).
        """
        return swiftpro_jacobian_pos4(self.q)

    def getEEJacobianFull(self):
        """
        4×4 Jacobian mapping dq → [dx, dy, dz, d(yaw)] in world_enu-aligned
        axes.  Use for combined position + EE yaw control tasks.
        """
        return swiftpro_jacobian_full4(self.q)