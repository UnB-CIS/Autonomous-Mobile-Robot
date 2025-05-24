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
noDetectionDist=0.5
maxDetectionDist=0.2
detect=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
braitenbergR=[-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
v0=2



while sim.getSimulationState() != sim.simulation_stopped:
    # # Get the position and orientation of the Pioneer robot
    pioneer_position = sim.getObjectPosition(robot, -1)
    pioneer_orientation = sim.getObjectOrientation(robot, -1)

    

    # Print the position and orientation
    print("Position:", pioneer_position)
    print("Orientation:", pioneer_orientation)
    for i in range(0,16):
        res, dist, _, _, _ = sim.readProximitySensor(usensors[i])

        if (res>0) and (dist<noDetectionDist):
            if (dist<maxDetectionDist):
                dist=maxDetectionDist
            detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
        else:
            detect[i]=0

    vLeft=v0
    vRight=v0

    for i in range(0,16):
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]

    # Setting Velocities
    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)

    #sim.step()


print("Simulation started")