"""closest_point controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, PositionSensor
import time
# create the Robot instance.
robot = Robot()
MAX_SPEED = 12.3
w_radius = 0.195/2
distance_from_wall = 0.50
len_between_wheels = 0.325
Lidar_robot = [0.08,00.21]
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# To rotate 180 degree
distance_travelled = len_between_wheels/2 * 3.14
linear_vel_max = MAX_SPEED * w_radius
time_req_pi = distance_travelled/linear_vel_max
rad = MAX_SPEED 

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

r_s = robot.getDevice('left wheel sensor')
l_s = robot.getDevice('right wheel sensor')
r_s.enable(timestep)
l_s.enable(timestep)


flag = 0
rotate = 0

def goal(d,angle):
#Returns the pose of the goal and the start time
    if angle <= 90:
        turn_angle = 90 - angle
    else:
        turn_angle = 90 + angle
    pos = turn_angle * 6.28 /180
    a = -6.28 + pos
    b = 6.28-pos
    r_m.setPosition(-6.28 + pos)
    l_m.setPosition(6.28-pos)
    return robot.getTime(), a, b, d
    
def move():
# Moves the robot linearly
    r_m.setPosition(float('inf'))
    l_m.setPosition(float('inf'))
    r_m.setVelocity(1.230)
    l_m.setVelocity(1.230)

def stop():
# Stops the robot
    r_m.setPosition(float('inf'))
    l_m.setPosition(float('inf'))
    r_m.setVelocity(0)
    l_m.setVelocity(0)

    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    scan = lidar.getRangeImage()

    if flag == 0 or min_val == 0:
    # Scans the first half
        min_val = min(scan)
        min_angle = scan.index(min(scan))
        flag = 1
        first_scan = robot.getTime()
        a_scan = [min_val, min_angle]
    elif flag == 1 and rotate == 0:
    # Rotates the robot and scans the second half
        r_m.setPosition(-6.28)
        l_m.setPosition(6.28)

        if robot.getTime() - first_scan > 2:
            new_scan = lidar.getRangeImage()
            new_min = min(new_scan)
            min_angle_r = new_scan.index(new_min)
            min_val_r = new_min
            rotate = 1
            flag = 2
            b_scan = [min_val_r, min_angle_r]
        
    elif flag == 2 and rotate == 1:
    # Finds the shortest distance from both scans and their respective angles
        print("The distance value and angle during scan of first half quadrant {} and {}".format(min_val,min_angle))
        print("The distance value and angle during scan of second half quadrant {} and {}".format(min_val_r,min_angle_r))
        if a_scan[0] > b_scan[0]:
            time_1,a, b, distance = goal(b_scan[0], b_scan[1])
        else:
            time_1,a, b, distance = goal(a_Scan[0], a_Scan[1])
        flag = 3
    
    elif flag == 3 and rotate == 1 and robot.getTime() - time_1 > 2 :
        selected_vel = 0.1 * MAX_SPEED
        Linear_wheel_vel = selected_vel * w_radius
        time_to_reach = (distance - distance_from_wall + Lidar_robot[0])/Linear_wheel_vel
        number_of_roatation = selected_vel * time_to_reach
        if l_s.getValue() < (a + number_of_roatation):
            print("Move")
            move()
        elif l_s.getValue() >= (a + number_of_roatation):
            print("stop")
            stop()
            flag = 4

    pass

# Enter here exit cleanup code.
