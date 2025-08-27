#!/usr/bin/env python3
import math

def ik_solver(x, y, z, r2=1.0, r3=1.0):
    solutions = []
    configs = [("plus", "plus"), ("plus", "minus"), ("minus", "plus"), ("minus", "minus")]

    for theta2_config, theta3_config in configs:
        # Step 1: theta1
        theta1 = math.atan2(y, x)

        # Project to x-y plane
        planar_dist = math.sqrt(x**2 + y**2)
        z_proj = z

        # Step 2: theta3 using Law of Cosines
        D = (planar_dist**2 + z_proj**2 - r2**2 - r3**2) / (2 * r2 * r3)
        if abs(D) > 1.0:
            # Not solvable, append invalid solution
            solutions.append(([theta1, None, None], False))
            continue

        theta3_mag = math.acos(D)
        theta3 = theta3_mag if theta3_config == "plus" else -theta3_mag

        # Step 3: theta2 using Law of Sines
        k1 = r2 + r3 * math.cos(theta3)
        k2 = r3 * math.sin(theta3)
        theta2_raw = math.atan2(z_proj, planar_dist) - math.atan2(k2, k1)
        theta2 = theta2_raw if theta2_config == "plus" else -theta2_raw

        # Check joint limits
        theta2_min, theta2_max = -math.pi/4, 3*math.pi/4
        theta3_min, theta3_max = -3*math.pi/4, 3*math.pi/4

        theta2_ok = theta2_min <= theta2 <= theta2_max
        theta3_ok = theta3_min <= theta3 <= theta3_max
        possible = theta2_ok and theta3_ok

        angles = [theta1, theta2, theta3]
        solutions.append((angles, possible))

    return solutions


def main():
    print("=== Inverse Kinematics of Anthropomorphic Arm ===")

    try:
        x = float(input("Enter x position of end-effector: "))
        y = float(input("Enter y position of end-effector: "))
        z = float(input("Enter z position of end-effector: "))
    except ValueError:
        print("Invalid input. Please enter float numbers.")
        return

    results = ik_solver(x, y, z)

    for angles, possible in results:
        print(f"Angles thetas solved ={angles} , solution possible = {possible}")


if __name__ == "__main__":
    main()
