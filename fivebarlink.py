import math
import numpy as np

def get_angle_difference(angle_1, angle_2):
    """
    Finds the minimum angular difference between two angles that are within the domain of 0 to 2*pi
    :param angle_1: First input
    :param angle_2: Second input
    :return: The difference between the two angles. If angle_2 is larger up to pi than angle_1, output is negative
    """
    difference = angle_1 - angle_2
    if difference > math.pi:
        difference = (difference - 2*math.pi)
    elif difference < -math.pi:
        difference = (difference + 2*math.pi)
    return difference

class Dynamics:
    def __init__(self, bottom_length, mass_matrix, length_matrix, a_length_matrix, x_hint, y_hint, state_hint, i_matrix=None):
        # mass
        self.M1, self.M2, self.M3, self.M4 = mass_matrix
        self.q1_hint = 0
        self.q2_hint = 0
        self.q3_hint = 0
        self.q4_hint = 0
        # Distance to center of mass from base joint
        self.L1, self.L2, self.L3, self.L4 = length_matrix
        # Lengths of each bar
        self.LB = bottom_length
        self.a1, self.a2, self.a3, self.a4 = a_length_matrix

        # Position hints for setting initial q3 and q4 angles to correct position
        self.x_hint = x_hint
        self.y_hint = y_hint
        self.state_hint = state_hint

        # Desired start position:
        hint_angle_options = self.inverse_kinematics(self.x_hint, self.y_hint)

        # Path
        self.path = None

        if hint_angle_options:
            hint_angles = hint_angle_options[self.state_hint]
            self.q1_hint = hint_angles[0] % (2 * math.pi)
            self.q2_hint = hint_angles[1] % (2 * math.pi)
            self.q3_hint = hint_angles[2] % (2 * math.pi)
            self.q4_hint = hint_angles[3] % (2 * math.pi)
        else:
            raise ValueError("x and y hint position are unreachable")

        def i_calc(m, a, length):
            return 1/12 * m * a * a + m * length * length

        if i_matrix is not None:
            self.I1, self.I2, self.I3, self.I4 = i_matrix
        else:
            self.I1 = i_calc(self.M1, self.a1, self.L1)
            self.I2 = i_calc(self.M2, self.a2, self.L2)
            self.I3 = i_calc(self.M3, self.a3, self.L3)
            self.I4 = i_calc(self.M4, self.a4, self.L4)

        self.g = 0

    def get_q3_q4(self, q1, q2, x_hint=None, y_hint=None, state_hint=None):
        if x_hint is not None and y_hint is not None and state_hint is not None:
            hint_angle_options = self.inverse_kinematics(x_hint, y_hint)
            if hint_angle_options:
                hint_angles = hint_angle_options[state_hint]
                self.q1_hint, self.q2_hint, self.q3_hint, self.q4_hint = [angle % (2 * math.pi) for angle in hint_angles]
                self.state_hint = state_hint
            else:
                raise ValueError("x and y hint position are unreachable")
        elif x_hint is not None or y_hint is not None or state_hint is not None:
            raise ValueError("Not enough inputs were provided, must provide at x, y and state or none")

        s_1, c_1 = math.sin(q1), math.cos(q1)
        s_2, c_2 = math.sin(q2), math.cos(q2)

        u_q1q2 = self.a2 * s_2 - self.a1 * s_1
        y_q1q2 = self.a2 * c_2 - self.a1 * c_1 + self.LB
        c_q1q2 = self.a3**2 - self.a4**2 - y_q1q2**2 - u_q1q2**2
        b_q1q2 = 2 * self.a4 * u_q1q2
        a_q1q2 = 2 * self.a4 * y_q1q2

        feasibility = a_q1q2**2 + b_q1q2**2 - c_q1q2**2
        if feasibility < 0:
            print("Entered singularity, recoverable?")
            feasibility = 0

        param = math.sqrt(feasibility)
        q4_1 = math.atan2(param, c_q1q2) + math.atan2(b_q1q2, a_q1q2) - q2
        q3_1 = math.atan2(u_q1q2 + self.a4 * math.sin(q2 + q4_1), y_q1q2 + self.a4 * math.cos(q2 + q4_1)) - q1

        param = -math.sqrt(feasibility)
        q4_2 = math.atan2(param, c_q1q2) + math.atan2(b_q1q2, a_q1q2) - q2
        q3_2 = math.atan2(u_q1q2 + self.a4 * math.sin(q2 + q4_2), y_q1q2 + self.a4 * math.cos(q2 + q4_2)) - q1

        q4_1, q3_1, q4_2, q3_2 = [angle % (2 * math.pi) for angle in [q4_1, q3_1, q4_2, q3_2]]

        error4_1 = get_angle_difference(q4_1, self.q4_hint)
        error4_2 = get_angle_difference(q4_2, self.q4_hint)
        error3_1 = get_angle_difference(q3_1, self.q3_hint)
        error3_2 = get_angle_difference(q3_2, self.q3_hint)

        first_con = (error4_1**2) + (error3_1**2)
        second_con = (error4_2**2) + (error3_2**2)
        self.q1_hint, self.q2_hint = q1, q2

        if first_con < second_con:
            self.q3_hint, self.q4_hint = q3_1, q4_1
            return q3_1, q4_1
        else:
            self.q3_hint, self.q4_hint = q3_2, q4_2
            return q3_2, q4_2

    def f(self, t, y):
        q1, q2, qd1, qd2, t1, t2 = y
        qd = np.array([[qd1], [qd2]])

        s_1, c_1 = math.sin(q1), math.cos(q1)
        s_2, c_2 = math.sin(q2), math.cos(q2)

        q3, q4 = self.get_q3_q4(q1, q2)

        s_13, s_24 = math.sin(q1 + q3), math.sin(q2 + q4)
        c_13, c_24 = math.cos(q1 + q3), math.cos(q2 + q4)

        phi11 = -self.a1 * s_1 - self.a3 * s_13
        phi12 = self.a2 * s_2 + self.a4 * s_24
        phi13 = -self.a3 * s_13
        phi14 = self.a4 * s_24
        phi21 = self.a1 * c_1 + self.a3 * c_13
        phi22 = -self.a2 * c_2 - self.a4 * c_24
        phi23 = self.a3 * c_13
        phi24 = -self.a4 * c_24

        phi1 = np.array([phi11, phi12, phi13, phi14])
        phi2 = np.array([phi21, phi22, phi23, phi24])
        phi3 = np.array([1, 0, 0, 0])
        phi4 = np.array([0, 1, 0, 0])

        phi = np.array([phi1, phi2, phi3, phi4])
        try:
            phi_inv = np.linalg.inv(phi)
        except np.linalg.LinAlgError:
            phi_inv = np.linalg.pinv(phi)

        selection_matrix = np.array([[0, 0], [0, 0], [1, 0], [0, 1]])
        rho = np.matmul(phi_inv, selection_matrix)

        qd_prime = np.matmul(rho, qd)
        qd3, qd4 = qd_prime[2][0], qd_prime[3][0]

        g_prime = self.g * np.array([
            [(self.M1 * self.L1 + self.M3 * self.a1) * c_1 + self.M3 * self.L3 * c_13],
            [(self.M2 * self.L2 + self.M4 * self.a2) * c_2 + self.M4 * self.L4 * c_24],
            [self.M3 * self.L3 * c_13],
            [self.M4 * self.L4 * c_24]
        ])
        rho_t = np.transpose(rho)
        g_comp = np.matmul(rho_t, g_prime)

        c_3, c_4 = math.cos(q3), math.cos(q4)

        d11 = self.M1 * self.L1**2 + self.M3 * (self.a1**2 + self.L3**2 + 2 * self.a1 * self.L3 * c_3) + self.I1 + self.I3
        d13 = self.M3 * (self.L3**2 + self.a1 * self.L3 * c_3) + self.I3
        d22 = self.M2 * self.L2**2 + self.M4 * (self.a2**2 + self.L4**2 + 2 * self.a2 * self.L4 * c_4) + self.I2 + self.I4
        d24 = self.M4 * (self.L4**2 + self.a2 * self.L4 * c_4) + self.I4
        d31, d33 = d13, self.M3 * self.L3**2 + self.I3
        d42, d44 = d24, self.M4 * self.L4**2 + self.I4

        d1 = np.array([d11, 0, d13, 0])
        d2 = np.array([0, d22, 0, d24])
        d3 = np.array([d31, 0, d33, 0])
        d4 = np.array([0, d42, 0, d44])

        d_prime = np.array([d1, d2, d3, d4])
        d_comp = np.matmul(rho_t, np.matmul(d_prime, rho))

        s_3, s_4 = math.sin(q3), math.sin(q4)

        h1 = -self.M3 * self.a1 * self.L3 * s_3
        h2 = -self.M4 * self.a2 * self.L4 * s_4

        c1 = np.array([h1 * qd3, 0, h1 * (qd1 + qd3), 0])
        c2 = np.array([0, h2 * qd4, 0, h2 * (qd2 + qd4)])
        c3 = np.array([-h1 * qd1, 0, 0, 0])
        c4 = np.array([0, -h2 * qd2, 0, 0])

        c_prime = np.array([c1, c2, c3, c4])

        phi_d11 = -self.a1 * c_1 * qd1 - self.a3 * c_13 * (qd1 + qd3)
        phi_d12 = self.a2 * c_2 * qd2 + self.a4 * c_24 * (qd2 + qd4)
        phi_d13 = -self.a3 * c_13 * (qd1 + qd3)
        phi_d14 = self.a4 * c_24 * (qd2 + qd4)

        phi_d21 = -self.a1 * s_1 * qd1 - self.a3 * s_13 * (qd1 + qd3)
        phi_d22 = self.a2 * s_2 * qd2 + self.a4 * s_24 * (qd2 + qd4)
        phi_d23 = -self.a3 * s_13 * (qd1 + qd3)
        phi_d24 = self.a4 * s_24 * (qd2 + qd4)

        phi_d1 = np.array([phi_d11, phi_d12, phi_d13, phi_d14])
        phi_d2 = np.array([phi_d21, phi_d22, phi_d23, phi_d24])
        phi_d3 = np.array([0, 0, 0, 0])
        phi_d4 = np.array([0, 0, 0, 0])

        phi_d = np.array([phi_d1, phi_d2, phi_d3, phi_d4])

        rho_d = -np.matmul(phi_inv, np.matmul(phi_d, rho))

        c_comp = np.matmul(rho_t, np.matmul(c_prime, rho)) + np.matmul(rho_t, np.matmul(d_prime, rho_d))

        tau = np.array([[t1], [t2]])
        qdd = np.matmul(np.linalg.inv(d_comp), (tau - np.matmul(c_comp, qd) - g_comp))

        return [qd1, qd2, qdd[0][0], qdd[1][0], 0, 0]

    def inverse_kinematics(self, x, y):
        """
        Calculates inverse kinematics for the 5-bar linkage.
        
        :param x: Desired x-coordinate of the end-effector
        :param y: Desired y--coordinate of the end-effector
        :return: List of possible solutions, each containing [q1, q2, q3, q4]
        """
        solutions = []
        
        # Calculate q2
        cos_q2 = (x**2 + y**2 - self.a1**2 - self.a2**2) / (2 * self.a1 * self.a2)
        if abs(cos_q2) > 1:
            return solutions  # No solution exists
        
        q2_1 = math.atan2(math.sqrt(1 - cos_q2**2), cos_q2)
        q2_2 = math.atan2(-math.sqrt(1 - cos_q2**2), cos_q2)
        
        for q2 in [q2_1, q2_2]:
            # Calculate q1
            k1 = self.a1 + self.a2 * math.cos(q2)
            k2 = self.a2 * math.sin(q2)
            q1 = math.atan2(y, x) - math.atan2(k2, k1)
            
            # Calculate q3 and q4
            q3, q4 = self.get_q3_q4(q1, q2)
            
            solutions.append([q1, q2, q3, q4])
        
        return solutions

# Example usage:
if __name__ == "__main__":
    # Example parameters (you may need to adjust these based on your specific robot)
    bottom_length = 0.5
    #Distance between bases
    mass_matrix = [1, 1, 1, 1]
    #Masses of each link
    length_matrix = [0.25, 0.25, 0.25, 0.25]
    #Lengths of links(from joint to center of mass)
    a_length_matrix = [0.5, 0.5, 0.5, 0.5]
    #Full lengths of links

    #Hints provide a sort of guess for the position
    #Should be implemented by setting last position euqal to hints 
    x_hint = 0.5
    y_hint = 0.5
    #Choses between the 4 different states(0,1,2,3)
    state_hint = 0

    dynamics = Dynamics(bottom_length, mass_matrix, length_matrix, a_length_matrix, x_hint, y_hint, state_hint)

    # Example inverse kinematics calculation

    x_desired = float(input("X-Coord"))
    y_desired = float(input("Y-Coord"))
    solutions = dynamics.inverse_kinematics(x_desired, y_desired)

    print(f"Inverse kinematics solutions for end-effector position ({x_desired}, {y_desired}):")
    for i, solution in enumerate(solutions):
        print(f"Solution {i + 1}:")
        print(f"  q1 = {math.degrees(solution[0]):.2f}°")
        print(f"  q2 = {math.degrees(solution[1]):.2f}°")
        print(f"  q3 = {math.degrees(solution[2]):.2f}°")
        print(f"  q4 = {math.degrees(solution[3]):.2f}°")

    # Example forward dynamics calculation
    '''
    t = 0
    y = [0, 0, 0, 0, 0, 0]  # Initial conditions: q1, q2, qd1, qd2, t1, t2
    result = dynamics.f(t, y)
    print("\nForward dynamics result:")
    print(f"qd1 = {result[0]:.4f}")
    print(f"qd2 = {result[1]:.4f}")
    print(f"qdd1 = {result[2]:.4f}")
    print(f"qdd2 = {result[3]:.4f}")'''