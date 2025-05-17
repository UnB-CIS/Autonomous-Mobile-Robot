from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")

sim.startSimulation()
pioneer = sim.getObject("/PioneerP3DX")

while True:
    # Get the position and orientation of the Pioneer robot
    pioneer_position = sim.getObjectPosition(pioneer, -1)
    pioneer_orientation = sim.getObjectOrientation(pioneer, -1)

    # Print the position and orientation
    print("Position:", pioneer_position)
    print("Orientation:", pioneer_orientation)


print("Simulation started")