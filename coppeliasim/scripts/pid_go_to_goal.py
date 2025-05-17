from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")

sim.startSimulation()
#sim.setStepping(True)

# Pioneer configs
robot = sim.getObject("/PioneerP3DX")
usensors = [(sim.getObject("/PioneerP3DX/ultrasonicSensor", {"index": i})) for i in range(0,16)]
motorLeft=sim.getObject("/PioneerP3DX/leftMotor")
motorRight=sim.getObject("/PioneerP3DX/rightMotor")
v0=0



while sim.getSimulationState() != sim.simulation_stopped:
    # # Get the position and orientation of the Pioneer robot
    # pioneer_position = sim.getObjectPosition(pioneer, -1)
    # pioneer_orientation = sim.getObjectOrientation(pioneer, -1)

    

    # Print the position and orientation
    # print("Position:", pioneer_position)
    # print("Orientation:", pioneer_orientation)

    vLeft=v0
    vRight=v0

    # Setting Velocities
    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)

    #sim.step()


print("Simulation started")