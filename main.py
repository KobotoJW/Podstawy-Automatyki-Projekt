import matplotlib.pyplot as plt
from balancing_robot import BalancingRobot
from pid import PID

robot = BalancingRobot()

controller = PID()
controller.setPoint(robot.body_angle)

deltat = 0.001  # time step
t = 0  # initial time
tf = 30  # final time

time = []
angle = []
speed = []
robot_speeds = []
robot_accelerations = []
setpoint_data = []

robot = BalancingRobot()
controller = PID(P=6, I=-0.2, D=750)

# set the setpoint for the PID controller
controller.setPoint(robot.body_angle)

# simulation loop
while t < tf:
    current_angle = robot.body_angle
    current_speed = robot.body_angle_speed
    current_robot_speed = robot.robot_speed
    setpoint_data.append(controller.getPoint())
    # update the PID controller with the current angle
    control_input = controller.update(current_angle)

    # use the f() method to update the robot's state
    body_angle_speed, body_angle_acceleration, robot_position, robot_speed, robot_acceleration = robot.f(robot.body_angle, robot.body_angle_speed, robot.robot_position, robot.robot_speed, deltat, control_input)
    
    robot.body_angle = robot.body_angle + body_angle_speed*deltat
    robot.body_angle_speed = robot.body_angle_speed + body_angle_acceleration*deltat
    robot.robot_speed = robot.robot_speed + robot_acceleration*deltat
    
    if robot.is_downed():
        print("Robot is downed at t = {:.2f}s".format(t))
        break

    time.append(t)
    angle.append(robot.body_angle)
    speed.append(robot.body_angle_speed)
    robot_accelerations.append(robot.robot_acceleration)
    robot_speeds.append(robot_speed)

    # increment the time
    t += deltat

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)
#add context line at 0 for each plot
ax1.axhline(0, color='red', linestyle='--')
ax1.set_title("Robot Angle")
ax1.set_ylabel("Angle (rad)")
ax1.plot(time, angle)

ax2.axhline(0, color='red', linestyle='--')
ax2.set_title("Pendulum Speed")
ax2.set_ylabel("Speed (rad/s)")
ax2.plot(time, speed)

ax3.axhline(0, color='red', linestyle='--')
ax3.set_title("Robot Acceleration")
ax3.set_ylabel("Acceleration (m/s^2)")
ax3.set_xlabel("Time (s)")
ax3.plot(time, robot_accelerations)

ax4.axhline(0, color='red', linestyle='--')
ax4.set_title("Robot Speed relative to ground")
ax4.set_ylabel("Speed (m/s)")
ax4.set_xlabel("Time (s)")
ax4.plot(time, robot_speeds)

plt.show()