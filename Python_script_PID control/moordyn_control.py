import moordyn
#from control import TransferFunction, pid, feedback, step_response, lqr
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.integrate import odeint

LEVEL_MSG = 2

# =============================================================================
# Controller.h
# =============================================================================

def initialize_moordyn_system(filepath):
    system = moordyn.Create(filepath)
    number_coupled_DOF = moordyn.NCoupledDOF(system)
    
    moordyn.SetVerbosity(system, LEVEL_MSG)
    moordyn.SetLogFile(system, "Mooring/lines_v2.log")
    moordyn.SetLogLevel(system, LEVEL_MSG)
    moordyn.Log(system, "We are ready to work!")
    
    return system

def initialize_positions_and_velocities():
    x = [-0.37, 0.120, -0.05,
         -0.37, -0.120, -0.05,
          0.37, 0.120, -0.05,
          0.37, -0.120, -0.05]
    
    v = [0.0, 0.0, 0.0,
         0.0, 0.0, 0.0,
         0.0, 0.0, 0.0,
         0.0, 0.0, 0.0]
    
    return x, v

def initialize_classes(system):
    class Vessel:
        def __init__(self, c, mass1, position_vessel, velocity_vessel):
            self.Body1 = moordyn.GetBody(system, 1)
            self.r1, self.u1 = moordyn.GetBodyState(self.Body1)
            self.r1= position_vessel
            self.u1= velocity_vessel
            self.Body_ID1 = moordyn.GetBodyID(self.Body1)
            self.mass1 = mass1
            self.c = c

        def vessel_position(self):
            self.r1, self.u1 = moordyn.GetBodyState(self.Body1)

        def vessel_xy_position(self):
            X_vessel = self.r1[0]
            Z_vessel = self.r1[2]

            return X_vessel, Z_vessel
        
    class Pendulum:
        def __init__(self, length, mass2, b, I, position_pendulum, velocity_pendulum):
            self.Body2 = moordyn.GetBody(system, 2)
            self.r2, self.u2 = moordyn.GetBodyState(self.Body2)
            self.r2 = position_pendulum
            self.u2 = velocity_pendulum
            self.Body_ID2 = moordyn.GetBodyID(self.Body2)
            self.mass2 = mass2
            self.length = length
            self.b = b
            self.I = I

        def update_position(self):
            self.r2, self.u2 = moordyn.GetBodyState(self.Body2)
    
        def calculate_theta(self):
            x = self.r2[0]  # Assuming x is the first element
            z = self.r2[2]  # Assuming z is the third element
            theta = math.atan2(z, x)  # Calculate theta using atan2 for proper quadrant
            return theta
    
    class Line:
        def __init__(self, l, index):
            self.line = moordyn.GetLine(system, index)
            self.line_length = moordyn.GetLineUnstretchedLength(self.line)
    
    class Node:
        def __init__(self, position, velocity, index):
            self.node = moordyn.GetPoint(system, index)
            self.node_ID = moordyn.GetPointID(self.node)
            self.node_pos = moordyn.GetPointPos(self.node)
            self.node_pos = position
            self.node_vel = moordyn.GetPointVel(self.node)
            self.node_vel = velocity

        def nodestaste(self):
            self.node_pos = moordyn.GetPointPos(self.node)
            self.node_vel = moordyn.GetPointVel(self.node)

        def calculate_nodestate(self):
            x = self.node_pos
            v = self.node_vel
            return x, v
        
    return Vessel, Pendulum, Line, Node

def pid_control_input(deltaT, error, previous_error, integral, theta_pos, previous_theta_pos):
    Kp = 196
    Kd = 0.014
    Ki = 71
    derivative = (error - previous_error) / deltaT
    integral += error * deltaT
    V_x = (Kp * error) + (Kd * derivative) + (Ki * integral)
    
    Kp_theta = 50
    Kd_theta = 200
    derivative_theta = (theta_pos - previous_theta_pos) / deltaT
    V_theta = (Kp_theta * theta_pos) + (Kd_theta * derivative_theta)
    
    V = V_x + V_theta
    V_max = 24
    V_min = -24

    V = np.clip(V, V_min, V_max)

    return V, integral

def dc_motor_with_load(V_s, deltaT, J_m, B_m, R_a, L_a, K_b, K_t, r_p, initial_omega=0, initial_theta=0, total_time=12=0):
    # Initialize variables
    time = [0]
    omega = [initial_omega]
    theta = [initial_theta]
    alpha = [0]
    I_a = [0]
    tension_cable = [0]

    # Time stepping using while loop
    t = deltaT
    while t <= total_time:
        # Compute current
        I_a_current = (V_s - K_b * omega[-1]) / R_a
        I_a.append(I_a_current)
        
        # Compute motor torque
        T_motor = K_t * I_a_current
        
        # Compute load torque
        T_load = T_motor
        
        # Compute angular acceleration
        alpha_current = (T_load - B_m * omega[-1]) / J_m
        alpha.append(alpha_current)
        
        # Update angular velocity
        omega_current = omega[-1] + alpha_current * deltaT
        omega.append(omega_current)
        
        # Update angular displacement
        theta_current = theta[-1] + omega_current * deltaT
        theta.append(theta_current)
        
        # Compute tension in cable
        tension_cable_current = T_load / r_p
        tension_cable.append(tension_cable_current)
        
        # Update time
        time.append(t)
        t += deltaT

    # Plot results
    plt.figure(figsize=(10, 6))
    
    plt.subplot(3, 1, 1)
    plt.plot(time, tension_cable, label='Tension in Cable')
    plt.xlabel('Time (s)')
    plt.ylabel('Tension (N)')
    plt.title('Tension in Cable vs Time')
    plt.legend()
    
    plt.subplot(3, 1, 2)
    plt.plot(time, theta, label='Angular Displacement')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Displacement (rad)')
    plt.title('Angular Displacement vs Time')
    plt.legend()
    
    plt.subplot(3, 1, 3)
    plt.plot(time, omega, label='Angular Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Angular Velocity vs Time')
    plt.legend()
    
    plt.tight_layout()
    plt.show()
    
    return np.array(time), np.array(tension_cable), np.array(theta), np.array(omega), np.array(alpha), np.array(I_a)

def calculate_tension_components(tension_cable, w, h):
    """Calculate the horizontal and vertical components of the tension."""
    T_v = w * h  # Vertical component due to weight
    T_h = math.sqrt(tension_cable**2 - T_v**2)  # Horizontal component from total tension
    return T_h, T_v

def apply_control_input(deltaT, V, w, h, r_p, system, lines):

    phi = np.degrees(np.arctan(0.5))
    alpha = 45.0
    d = np.sqrt(0.37**2 + 0.120**2)
    T = np.array([
        [-np.cos(alpha), -np.cos(alpha), np.cos(alpha), np.cos(alpha)],
        [np.sin(alpha), -np.sin(alpha), np.sin(alpha), -np.sin(alpha)],
        [d * np.sin(alpha - phi), -d * np.sin(alpha - phi), -d * np.sin(alpha - phi), d * np.sin(alpha - phi)]
    ])
    
    U, S, VT = np.linalg.svd(T)
    S_inv = np.zeros((4, 3))
    S_inv[:3, :3] = np.diag(1/S[:3])
    V_d = np.array([[V], [0], [0]])
    V_x = VT.T @ S_inv @ U.T @ V_d

    for i in range(4):
        Vref = V_x[i]
        time, tension_cable, theta, omega, alpha, I_a = dc_motor_with_load(
            Vref, deltaT, J_m=0.01, B_m=0.1, R_a=1, L_a=0.5, K_b=0.1, K_t=0.1, r_p=0.02
        )

        tension = tension_cable[-1]  # Final tension in the cable
        
        # Calculate the horizontal component of the tension
        T_h, T_v = calculate_tension_components(tension, w, h)  # Horizontal and vertical components

        F_i = T_h  # Horizontal component for the force
        Cable_v = omega[-1] * r_p  # Cable velocity
       
        Ls = h * math.sqrt((2 * (abs(F_i) / w * h) + 1))
        x = (abs(F_i) / w) * math.acosh((w * h / abs(F_i)) + 1)
        X = h * math.sqrt((2 * (abs(F_i) / w * h) + 1)) + x
        deltaX = X - x
        L_set = Ls + deltaX
        moordyn.SetLineUnstretchedLength(lines[i].line, L_set)
        moordyn.SetLineUnstretchedLengthVel(lines[i].line, Cable_v)

def PD_error(pendulum):
    theta_pos = pendulum.calculate_theta()
    return theta_pos

def find_error(vessel, position_reference,X_pos,Z_pos):
    current_pos =np.array([X_pos, Z_pos])
    error = position_reference - current_pos
    return error

def plot_graphs(times, angular_Position, position_X_vessel,position_Z_vessel, force):
    plt.subplot(4, 1, 1)
    plt.plot(times, angular_Position, '-b')
    plt.ylabel('Theta')
    plt.xlabel('Time')
    
    plt.subplot(4, 1, 2)
    plt.plot(times, force, '-b')
    plt.ylabel('Force')
    plt.xlabel('Time')
    
    plt.subplot(4, 1, 3)
    plt.plot(times, position_X_vessel, '-b')
    plt.ylabel('X')
    plt.xlabel('Time')

    plt.subplot(4, 1, 3)
    plt.plot(times, position_Z_vessel, '-b')
    plt.ylabel('Z')
    plt.xlabel('Time')
    
    plt.show()

def write_status_file(t, error, theta_pos):
    status_file_path = "simulation_status.txt"
    with open(status_file_path, "a") as status_file:
        status_file.write(f"Time: {t:.4f}, Error: {error}, Theta: {theta_pos}\n")

def external_condition(forces, threshold=3000, t=None):
    for i, force in enumerate(forces):
        force_magnitude = np.linalg.norm(force)
        if force_magnitude > threshold:
            if t is not None:
                print(f"Simulation stopped at time {t}. Force threshold exceeded at coupling point {i+1} with magnitude {force_magnitude}.")
            return False
    return True

def main(control_technique):

    # Initialize MoorDyn system
    filepath = "/home/nkhadi001/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/background/Mooring/lines_v2_point.txt"
    system = moordyn.Create(filepath)
    
    # Initialize positions and velocities
    x, v = initialize_positions_and_velocities()
    moordyn.Init(system, x, v)
    
    Vessel, Pendulum, Line, Node = initialize_classes(system)
    
    vessel = Vessel(c=1.1, mass1=6, position_vessel=[0, 0, -0.03], velocity_vessel=[0, 0, 0])
    pendulum = Pendulum(length=0.45, mass2=0.5, b=1.3, I=10, position_pendulum=[0, 0, -0.425], velocity_pendulum=[0, 0, 0])

    # Initialize nodes
    nodes = [Node(position=x[i*3:(i+1)*3], velocity=v[i*3:(i+1)*3], index=i+1) for i in range(4)]

    ref_point = [1.0, 0.0]
    position_reference = np.array(ref_point)
    lines = [Line(l, i+1) for i, l in enumerate(range(1, 5))]
    
    F = 0
    deltaT = 0.000005
    total_time = 120
    times = []
    position_X_vessel = []
    position_Z_vessel = []
    angular_Position = []
    force = []

    # Control parameters
    error = 0
    previous_error = 0
    integral = 0
    theta_pos = 0
    previous_theta_pos = 0
    
    t = 0
    while t < total_time:
        times.append(t)
        
        # Update states
        vessel.vessel_position()
        pendulum.update_position()
        for node in nodes:
            node.update_state()

        # Retrieve combined positions and velocities
        x_combined = []
        v_combined = []
        for node in nodes:
            x_node, v_node = node.calculate_nodestate()
            x_combined.extend(x_node)
            v_combined.extend(v_node)
        
        # Perform a time step and get forces
        forces = moordyn.Step(system, x_combined, v_combined, t, deltaT)
        
        # Check external conditions
        # Placeholder values for forces at four coupling points
        forces = [np.array([0.0, 0.0, 0.0]),
                  np.array([0.0, 0.0, 0.0]),
                  np.array([0.0, 0.0, 0.0]),
                  np.array([0.0, 0.0, 0.0])]

        if not external_condition(forces, threshold=3000, t=t):
            break
        
        if control_technique == 'PID':
            F, integral = pid_control_input(deltaT, error, previous_error, integral, theta_pos, previous_theta_pos)
         
        apply_control_input(deltaT, F, w=0.025, h=1.0, r_p=0.02, system=system, lines=lines)

        X_pos, Z_pos = vessel.vessel_xy_position()
        error = find_error(vessel, position_reference, X_pos, Z_pos)

        print(f"Positional Error: {error}")
        
        theta_pos = PD_error(pendulum)
        print(f"Positional Error: {theta_pos}")

        previous_error = error
        previous_theta_pos = theta_pos

        position_X_vessel.append(X_pos)
        position_Z_vessel.append(Z_pos)
        angular_Position.append(theta_pos)
        force.append(F)

        # Log status
        write_status_file(t, error, theta_pos)

        # Update time
        t += deltaT
    
    moordyn.Close(system)
    plot_graphs(times, angular_Position, position_X_vessel, position_Z_vessel, force)

if __name__ == "__main__":
    main('PID')