# ~/catkin_ws/src/antropomorphic_project/scripts/generate_matrices.py

#!/usr/bin/env python3
import sympy as sp

class DHMatrixGenerator:
    def __init__(self):
        # Joint symbols
        self.theta1, self.theta2, self.theta3 = sp.symbols('theta1 theta2 theta3')
        # DH parameters
        self.d = [0, 0, 0]
        self.a = [0, 1.0, 1.0]
        self.alpha = [sp.pi/2, 0, 0]
        self.theta = [self.theta1, self.theta2, self.theta3]

    def dh_transform(self, theta, d, a, alpha):
        return sp.Matrix([
            [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
            [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
            [0, sp.sin(alpha), sp.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def generate(self):
        A0_1 = self.dh_transform(self.theta[0], self.d[0], self.a[0], self.alpha[0])
        A1_2 = self.dh_transform(self.theta[1], self.d[1], self.a[1], self.alpha[1])
        A2_3 = self.dh_transform(self.theta[2], self.d[2], self.a[2], self.alpha[2])
        A0_3 = sp.simplify(A0_1 * A1_2 * A2_3)

        self.save_matrix_image(A0_1, 'A0_1.png')
        self.save_matrix_image(A1_2, 'A1_2.png')
        self.save_matrix_image(A2_3, 'A2_3.png')
        self.save_matrix_image(A0_3, 'A0_3.png')
        self.save_matrix_image(sp.simplify(A0_3), 'A0_3_simplified.png')

    def save_matrix_image(self, matrix, filename):
        from sympy import preview
        preview(matrix, viewer='file', filename=filename, dvioptions=["-D", "150"])

if __name__ == "__main__":
    gen = DHMatrixGenerator()
    gen.generate()
