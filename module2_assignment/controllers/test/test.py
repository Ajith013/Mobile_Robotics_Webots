"""test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
lidar = robot.getDevice('Sick LMS 291')
lidar.enable(60)

min_val = 20
min_angle = 360

r_m = robot.getDevice('right wheel')
l_m = robot.getDevice('left wheel')
flag = 0
rotate = 0
data = []



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    l_m.setPosition(float('inf'))
    r_m.setPosition(float('inf'))
    
    r_m.setVelocity(0.4)
    l_m.setVelocity(0.4)
    
    scan = lidar.getRangeImage()
    new = min(scan)
    print(new)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass
if scan_4[0] > 0.4463 and scan_4[180] > 0.4463:
            print("In loop 1")
            rotate_wall("c")
        elif scan_4[0] > 0.5 and scan_4[180] > 0.4463 and scan_4[90] > (0.50-Lidar_robot[0]):
            stop()
        elif scan_4[0] <= 0.4463 and scan_4[0] > 0.4:
            if scan_4[90] > (0.50-Lidar_robot[0]):
                move()
            elif scan_4[90] < (0.50-Lidar_robot[0]):
                rotate_wall("c")
        elif scan_4[0] > 0.4463 and scan_4[0] <= 0.5 and scan_4[90] > (0.50-Lidar_robot[0]):
            rotate_wall("a")
        elif scan_4[0] < 0.4:
            rotate_wall("c")
            print("In loop 3")
# Enter here exit cleanup code.
