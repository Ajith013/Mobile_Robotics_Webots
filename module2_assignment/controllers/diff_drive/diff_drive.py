"""diff_drive controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
MAX_SPEED = 12.3
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
r_m = robot.getDevice('right wheel')
l_m = robot.getDevice('left wheel')

r_s = robot.getDevice('left wheel sensor')
l_s = robot.getDevice('right wheel sensor')
r_s.enable(timestep)
l_s.enable(timestep)

l_m.setPosition(float('inf'))
r_m.setPosition(float('inf'))

# set up the motor speeds at 10% of the MAX_SPEED.
def set_speed(flag):
    if flag == 1:
        l_m.setVelocity(0.1 * MAX_SPEED)
        r_m.setVelocity(0.1 * 1.388 * MAX_SPEED)
    elif flag == 0:
        r_m.setVelocity(0.1 * MAX_SPEED)
        l_m.setVelocity(0.1 * 1.388 * MAX_SPEED)
    else:
        l_m.setVelocity(0)
        r_m.setVelocity(0)        
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # Process sensor data here.
    val = r_s.getValue()
    if val >=27.7183 and val <=65.8:
        set_speed(0)
    elif val < 27.7183:
        set_speed(1)
    else:
        set_speed(2)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
