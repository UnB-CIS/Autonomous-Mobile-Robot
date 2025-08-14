from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pid import PID
import numpy as np

class PioneerP3DX():
    def __init__(self, sim, target="/Target"):

        self.sim = sim

        #Robot info
        self.robot = sim.getObject("/PioneerP3DX")
        self.leftMotor = sim.getObject("/PioneerP3DX/leftMotor")
        self.rightMotor = sim.getObject("/PioneerP3DX/rightMotor")

        self.x, self.y, _ = sim.getObjectPosition(self.robot, -1)
        _, _, self.phi = sim.getObjectOrientation(self.robot, -1)

        self.wheel_radius = 0.19502
        self.wheels_distance = 0.331
        self.vLeft = 0
        self.vRight = 0

        #Control info
        self.angular_kp = 1
        self.angular_ki = 0.01
        self.angular_kd = 0.1

        self.linear_kp = 0.5
        self.linear_ki = 0
        self.linear_kd = 0

        self.max_wheel_velocity = 1

        self.angular_pid = PID(self.angular_kp, self.angular_ki, self.angular_kd) #OBS: Talvez as instâncias de PID possam ser passadas como parametros
        self.linear_pid = PID(self.linear_kp, self.linear_ki, self.linear_kd)

        #Target info
        self.target = sim.getObject(target) #Aqui eu assumo que o target é um objeto do coppelia, mas podemos trocar isso por um valor de coordenda.
        

    def step(self):

        self.x_pioneer, self.y_pioneer, _ = self.sim.getObjectPosition(self.robot, -1)
        _, _, self.phi_pioneer = self.sim.getObjectOrientation(self.robot, - 1)
        x_target, y_target, _ = self.sim.getObjectPosition(self.target, -1)

        delta_y = y_target - self.y_pioneer
        delta_x = x_target - self.x_pioneer
        delta_distance = np.sqrt(delta_x**2 + delta_y**2)

        phi_target = np.arctan2(delta_y, delta_x)
        delta_phi = phi_target - self.phi_pioneer 
        delta_phi = np.arctan2(np.sin(delta_phi), np.cos(delta_phi))

        angular_velocity = self.angular_pid.control(delta_phi) #Velocidade angular do robo e não das rodas
        speed = self.linear_pid.control(delta_distance) #Velocidade linear do robô

        self.vLeft= (2 * speed - angular_velocity * self.wheels_distance)/(2 * self.wheel_radius) 
        self.vRight= (2 * speed + angular_velocity * self.wheels_distance)/(2 * self.wheel_radius) 

        self.sim.setJointTargetVelocity(self.leftMotor, self.vLeft)
        self.sim.setJointTargetVelocity(self.rightMotor, self.vRight)




#Use example:
client = RemoteAPIClient()
sim = client.require("sim")

sim.startSimulation()
sim.setStepping(True)

robot = PioneerP3DX(sim)
while sim.getSimulationState() != sim.simulation_stopped:
    robot.step()
    sim.step()