from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.e_prev = 0
        self.e_acc = 0

    def control(self, e): 

        self.e_acc += e

        P = self.kp * e
        I = self.ki * self.e_acc
        D = self.kd * (e - self.e_prev)

        self.e_prev = e
        return P + I + D

client = RemoteAPIClient()
sim = client.require("sim")

sim.startSimulation()
sim.setStepping(True)

# Pioneer configs
robot = sim.getObject("/PioneerP3DX")
usensors = [(sim.getObject("/PioneerP3DX/ultrasonicSensor", {"index": i})) for i in range(0,16)]
motorLeft=sim.getObject("/PioneerP3DX/leftMotor")
motorRight=sim.getObject("/PioneerP3DX/rightMotor")
R = 0.19502
L = 0.331

# Controller Parameters
v_max_linear = 0.5
kp = 1.1              # Proportional coefficient
ki = 0.001              # Integrative coefficient
kd = 0.001              # Derivative coefficient
k_linear_vel_damping = 1.0 # Ganho para reduzir velocidade perto do alvo
max_wheel_velocity = 5 # Ex: 5.0 rad/s

# Target parameters
target = sim.getObject("/Target")

pid = PID(kp, ki, kd)

while sim.getSimulationState() != sim.simulation_stopped:

    x_desired, y_desired, _ = sim.getObjectPosition(target, -1)
    
    #print("Target position:", (x_desired, y_desired))

    # # Get the position and orientation of the Pioneer robot
    x, y, _ = sim.getObjectPosition(robot, -1)
    _, _, phi = sim.getObjectOrientation(robot, -1)
    
    # Print the position and orientation
    # print("Position:", (x, y))
    # print("Orientation:", phi)

    # Distância e ângulo para o alvo
    delta_x = x_desired - x
    delta_y = y_desired - y
    distance_to_target = np.sqrt(delta_x**2 + delta_y**2)

    phi_desired = np.arctan2(delta_y, delta_x)

    angle_error = phi_desired - phi
    angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

    omega = pid.control(angle_error)

    v_current_linear = min(v_max_linear, k_linear_vel_damping * distance_to_target)

    vLeft= (2*v_current_linear - omega*L)/(2*R) 
    vRight= (2*v_current_linear + omega*L)/(2*R) 

    vLeft = np.clip(vLeft, -max_wheel_velocity, max_wheel_velocity)
    vRight = np.clip(vRight, -max_wheel_velocity, max_wheel_velocity)

    # Setting Velocities
    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)

    sim.step()