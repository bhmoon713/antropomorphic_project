#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys
import math
import rospy
from geometry_msgs.msg import Vector3
import rospkg
# --- Import helpers  ---
# from antropomorphic_project.move_joints import JointMover
# from antropomorphic_project.rviz_marker import MarkerBasics

# try:
#     from .rviz_marker import MarkerBasics   # relative import (inside same package dir)
# except Exception:
#     from rviz_marker import MarkerBasics    # fallback to plain import (if run as script)

# try:
#     from .move_joints import JointMover   # relative import (inside same package dir)
# except Exception:
#     from move_joints import JointMover    # fallback to plain import (if run as script)

# --- Force Python to import our helpers from this package's src/ ---
_pkg_path = rospkg.RosPack().get_path('antropomorphic_project')
_src_path = os.path.join(_pkg_path, 'src')
if _src_path not in sys.path:
    sys.path.insert(0, _src_path)

from move_joints import JointMover
from rviz_marker import MarkerBasics


# /ee_pose_commands message (from the course)
from planar_3dof_control.msg import EndEffector


# ---------- Joint limits (Task 3) ----------
THETA2_MIN = -math.pi / 4.0      # -π/4
THETA2_MAX =  3.0 * math.pi / 4.0
THETA3_MIN = -3.0 * math.pi / 4.0
THETA3_MAX =  3.0 * math.pi / 4.0

def within_limits(angle, amin, amax):
    return (amin - 1e-12) <= angle <= (amax + 1e-12)

def wrap_pi(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ik_one(Px, Py, Pz, r2, r3, theta2_config, theta3_config):
    """
    Solve one IK branch (position-only) using the DH setup from the project:
      α1 = π/2, α2 = α3 = 0, d's = 0, links along x: r2, r3.

    theta2_config: 'plus'  -> use R (elbow-down style), θ1 = atan2(y,x)
                   'minus' -> use -R (base-π variant), θ1 = atan2(y,x) - π
    theta3_config: 'plus'  -> sin(θ3) >= 0
                   'minus' -> sin(θ3) <= 0
    Returns (thetas[list], possible[bool]).
    """
    R_mag = math.hypot(Px, Py)
    R_eff_sign = +1 if theta2_config == "plus" else -1
    R_eff = R_eff_sign * R_mag

    theta1 = math.atan2(Py, Px) if R_eff_sign > 0 else (math.atan2(Py, Px) - math.pi)

    # Law of cosines for planar 2-link in (R, Z)
    C3 = (R_mag*R_mag + Pz*Pz - r2*r2 - r3*r3) / (2.0 * r2 * r3)
    if C3 < -1.0 - 1e-12 or C3 > 1.0 + 1e-12:
        return [wrap_pi(theta1), float('nan'), float('nan')], False

    C3 = max(-1.0, min(1.0, C3))
    s3_abs = math.sqrt(max(0.0, 1.0 - C3*C3))
    S3 = +s3_abs if theta3_config == "plus" else -s3_abs
    theta3 = math.atan2(S3, C3)

    k1 = r2 + r3*C3
    k2 = r3*S3
    theta2 = math.atan2(Pz, R_eff) - math.atan2(k2, k1)

    # Normalize joints
    theta1 = wrap_pi(theta1)
    theta2 = wrap_pi(theta2)
    theta3 = wrap_pi(theta3)

    possible = within_limits(theta2, THETA2_MIN, THETA2_MAX) and \
               within_limits(theta3, THETA3_MIN, THETA3_MAX)

    return [theta1, theta2, theta3], possible


class EndEffectorMover(object):
    """
    - Subscribes to /ee_pose_commands (EndEffector): desired x,y,z + elbow policy hint
    - Subscribes to /end_effector_real_pose: latest achieved x,y,z
    - Drives joints via JointMover
    - Publishes target marker via MarkerBasics
    """

    def __init__(self):
        # Parameters (link lengths); defaults per project
        self.r2 = rospy.get_param("~r2", 1.0)
        self.r3 = rospy.get_param("~r3", 1.0)

        # Helpers
        rospy.loginfo("Creating JointMover and MarkerBasics…")
        self.joint_mover = JointMover()
        self.marker = MarkerBasics()

        # State
        self.last_real = Vector3()
        self.have_real = False
        self.marker_index = 0

        # Subscribers
        self.ee_cmd_sub = rospy.Subscriber(
            "/ee_pose_commands", EndEffector, self.ee_cmd_cb, queue_size=10
        )
        self.ee_real_sub = rospy.Subscriber(
            "/end_effector_real_pose", Vector3, self.ee_real_cb, queue_size=10
        )

        rospy.loginfo("EndEffectorMover READY (r2=%.3f, r3=%.3f)", self.r2, self.r3)

    def ee_real_cb(self, msg: Vector3):
        self.last_real = msg
        self.have_real = True

    @staticmethod
    def _ordered_configs(elbow_policy: str):
        """
        Map the hint string to a preferred ordering of (theta2_config, theta3_config).
        Accepts: "plus-plus", "plus-minus", "minus-plus", "minus-minus".
        Falls back to default order if unknown.
        """
        all_cfgs = [("plus", "plus"), ("plus", "minus"), ("minus", "plus"), ("minus", "minus")]
        elbow_policy = (elbow_policy or "").strip().lower()
        if elbow_policy in {"plus-plus", "plus-minus", "minus-plus", "minus-minus"}:
            pref = tuple(elbow_policy.split("-"))
            # Put preferred first, followed by the rest
            return [pref] + [c for c in all_cfgs if c != pref]
        return all_cfgs

    def ee_cmd_cb(self, msg: EndEffector):
        # Desired point from the ellipse generator
        x = float(msg.ee_xy_theta.x)
        y = float(msg.ee_xy_theta.y)
        z = float(msg.ee_xy_theta.z)
        elbow_policy = msg.elbow_policy.data if hasattr(msg.elbow_policy, "data") else str(msg.elbow_policy)

        # Publish goal marker (colored sphere)
        self.marker.publish_point(x, y, z, index=self.marker_index)
        self.marker_index += 1

        # Choose IK branch order based on hint
        for t2cfg, t3cfg in self._ordered_configs(elbow_policy):
            thetas, ok = ik_one(x, y, z, self.r2, self.r3, t2cfg, t3cfg)
            if ok:
                # Command joints
                self.joint_mover.move_all_joints(*thetas)
                # Optional: log current position error if available
                if self.have_real:
                    ex = x - self.last_real.x
                    ey = y - self.last_real.y
                    ez = z - self.last_real.z
                    err = math.sqrt(ex*ex + ey*ey + ez*ez)
                    rospy.loginfo_throttle(1.0,
                        "[IK %s-%s] goal=[%.3f, %.3f, %.3f] "
                        "real=[%.3f, %.3f, %.3f] |err|=%.3f -> thetas=%s",
                        t2cfg, t3cfg, x, y, z,
                        self.last_real.x, self.last_real.y, self.last_real.z,
                        err, [round(a, 6) for a in thetas]
                    )
                return

        # If no feasible solution is found
        rospy.logwarn_throttle(1.0,
            "No feasible IK found for goal [%.3f, %.3f, %.3f] (policy=%s). "
            "Check limits or link lengths.", x, y, z, elbow_policy
        )


def main():
    rospy.init_node("antropomorphic_end_effector_mover")
    mover = EndEffectorMover()
    rospy.loginfo("antropomorphic_end_effector_mover spinning…")
    rospy.spin()


if __name__ == "__main__":
    main()
