#!/usr/bin/env python3

import sys
import os
import sympy as sp 

# Add the correct path to import generate_matrixes from src/
pkg_src_path = os.path.join(os.path.dirname(__file__), "..", "src")
sys.path.insert(0, os.path.abspath(pkg_src_path))

from generate_matrixes import DHMatrixGenerator


def main():
    print("=== Forward Kinematics of Anthropomorphic Arm ===")
    
    # Prompt user input
    try:
        theta1 = float(input("Enter the value for theta_1 (in radians): "))
        theta2 = float(input("Enter the value for theta_2 (in radians): "))
        theta3 = float(input("Enter the value for theta_3 (in radians): "))
    except ValueError:
        print("Invalid input. Please enter valid float numbers.")
        return

    # Use the DHMatrixGenerator class from Task 1
    generator = DHMatrixGenerator()
    
    # Evaluate transformation matrix from base to end-effector
    A0_3_evaluated = generator.evaluate_a03(theta1, theta2, theta3)

    # Extract position (last column, first 3 rows)
    position = A0_3_evaluated[:3, 3]

    # Extract orientation (top-left 3x3 block)
    orientation = A0_3_evaluated[:3, :3]

    # Print position and orientation
    # print("\n=== Position Matrix (x, y, z) ===")
    # sp.pprint(position)

    # print("\n=== Orientation Matrix (3x3) ===")
    # sp.pprint(orientation)

    print("\nPosition Matrix:")
    print(position)

    print("\nOrientation Matrix:")
    print(orientation)


    # Save evaluated matrix as PNG
    generator.save_matrix_image(A0_3_evaluated, 'A03_simplify_evaluated.png')
    print("\n Saved matrix as A03_simplify_evaluated.png")

if __name__ == '__main__':
    main()
