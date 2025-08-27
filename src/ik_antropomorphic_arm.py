#!/usr/bin/env python3
import math

# ---------- Joint limits ----------
THETA2_MIN = -math.pi/4      # -π/4
THETA2_MAX =  3*math.pi/4    #  3π/4
THETA3_MIN = -3*math.pi/4    # -3π/4
THETA3_MAX =  3*math.pi/4    #  3π/4

# Link lengths (edit if needed)
R2 = 1.0
R3 = 1.0

def within_limits(angle, amin, amax):
    return (amin - 1e-12) <= angle <= (amax + 1e-12)

def wrap_pi(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def ik_one(Px, Py, Pz, r2, r3, theta2_config, theta3_config):
    """
    Solve one IK branch (position-only).
    theta2_config: 'plus' (elbow-down style) or 'minus' (elbow-up base-π variant)
    theta3_config: 'plus' (sin θ3 >= 0) or 'minus' (sin θ3 <= 0)
    """
    R_mag = math.hypot(Px, Py)
    R_eff_sign = +1 if theta2_config == "plus" else -1
    R_eff = R_eff_sign * R_mag

    theta1_base = math.atan2(Py, Px) if R_eff_sign > 0 else (math.atan2(Py, Px) - math.pi)

    C3 = (R_mag*R_mag + Pz*Pz - r2*r2 - r3*r3) / (2.0 * r2 * r3)
    if C3 < -1.0 - 1e-12 or C3 > 1.0 + 1e-12:
        return [wrap_pi(theta1_base), float('nan'), float('nan')], False

    C3 = max(-1.0, min(1.0, C3))
    s3_abs = math.sqrt(max(0.0, 1.0 - C3*C3))
    S3 = +s3_abs if theta3_config == "plus" else -s3_abs
    theta3 = math.atan2(S3, C3)

    k1 = r2 + r3*C3
    k2 = r3*S3
    theta2 = math.atan2(Pz, R_eff) - math.atan2(k2, k1)

    # --- Normalize all joints to [-π, π] ---
    theta1 = wrap_pi(theta1_base)
    theta2 = wrap_pi(theta2)
    theta3 = wrap_pi(theta3)

    # Joint-limit checks on normalized angles
    possible = True
    if not within_limits(theta2, THETA2_MIN, THETA2_MAX):
        possible = False
    if not within_limits(theta3, THETA3_MIN, THETA3_MAX):
        possible = False

    return [theta1, theta2, theta3], possible


def main():
    try:
        x = float(input("Enter Pee_x: "))
        y = float(input("Enter Pee_y: "))
        z = float(input("Enter Pee_z: "))
    except ValueError:
        print("Invalid input. Please enter numeric values.")
        return

    print(f"\nFor the Position P3 = [{x}, {y}, {z}]")

    # Enumerate in the same order as your example
    configs = [("plus", "plus"), ("plus", "minus"), ("minus", "plus"), ("minus", "minus")]

    for t2cfg, t3cfg in configs:
        thetas, ok = ik_one(x, y, z, R2, R3, t2cfg, t3cfg)
        print(f"Angles thetas solved ={thetas} , solution possible = {ok}")

if __name__ == "__main__":
    main()
