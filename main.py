import matplotlib.pyplot as plt
from balancing_robot import BalancingRobot
from pid import PID

# create an instance of the robot
robot = BalancingRobot()

# create an instance of the PID controller
controller = PID()
controller.setPoint(robot.phi)

# set the initial conditions for the simulation
deltat = 0.01  # time step
t = 0  # initial time
tf = 20  # final time
# create lists to store time, angle and speed data
time = []
angle = []
speed = []
robot_speed = []
setpoint_data = []

# initialize the robot and the PID controller
robot = BalancingRobot()
controller = PID(P=12, I=0, D=100, Derivator=0, Integrator=0, Integrator_max=3, Integrator_min=-3, Kp_speed = 0)

# set the setpoint for the PID controller
controller.setPoint(0)

speed_sum = 0

# simulation loop
while t < tf:
    # get the current angle of the pendulum
    current_angle = robot.phi
    
    # get the current speed of the pendulum
    current_speed = robot.phip
    
    # get the current speed of the robot relative to the ground
    current_robot_speed = robot.xp

    # append the current setpoint to the list
    setpoint_data.append(controller.getPoint())

    # update the PID controller with the current angle
    control_input = controller.update(current_angle)

    speed_sum += robot.xp
    control_input += -controller.Kp_speed*speed_sum

    # use the f() method to update the robot's state
    phip, phipp, xpp, xp = robot.f(robot.phi, robot.phip, robot.x, robot.xp, deltat, control_input)
    robot.phi = robot.phi + phip*deltat
    robot.phip = robot.phip + phipp*deltat
    robot.xp = robot.xp + xpp*deltat
    
    if robot.is_downed():
        print("Robot is downed at t = {:.2f}s".format(t))
        break

    # append the current time, angle, speed, and robot speed to the lists
    time.append(t)
    angle.append(robot.phi)
    speed.append(robot.phip)
    robot_speed.append(robot.xp)

    # increment the time
    t += deltat

fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)

ax1.set_title("Robot Angle")
ax1.set_ylabel("Angle (rad)")
ax1.plot(time, angle)
#ax1.plot(time, setpoint_data, linestyle='--', color='red')

ax2.set_title("Pendulum Speed")
ax2.set_ylabel("Speed (rad/s)")
ax2.plot(time, speed)

ax3.set_title("Robot Speed relative to ground")
ax3.set_ylabel("Speed (m/s)")
ax3.set_xlabel("Time (s)")
ax3.plot(time, robot_speed)

plt.show()