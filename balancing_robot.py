from math import cos
from math import sin
from math import pi

class BalancingRobot:
    def __init__(self):
        # variables for the model
        self.Mb = None  # kg      mass of main body (pendulum)
        self.Mw = None  # kg      mass of wheels
        self.d = None   # m       center of mass from base
        self.Ix = None  # kg.m^2  Moment of inertia of body robot_position-axis
        self.Iz = None  # kg.m^2  Moment of inertia of body z-axis
        self.g = None   # m/s^2   Acceleration due to gravity

        # variables for dynamic evaluation
        self.body_angle = None    # angle of the pendulum
        self.body_angle_speed = None   # angular speed of the pendulum
        self.body_angle_acceleration = None  # angular acceleration of the pendulum
        self.robot_position = None      # robot_position position of the robot
        self.robot_speed = None     # speed of the robot
        self.robot_acceleration = None    # acceleration of the robot

        self.initRobot()

    def initRobot(self):
        self.Mb = 13.3    # kg      mass of main body (pendulum)
        self.Mw = 1.5     # kg      mass of wheels
        self.d = 0.03     # m       center of mass from base
        self.Ix = 0.1935  # kg.m^2  Moment of inertia of body robot_position-axis
        self.Iz = 0.3379  # kg.m^2  Moment of inertia of body z-axis
        self.g = 9.81     # m/s^2   Acceleration due to gravity  

        self.body_angle = 2*pi/180  # the pendulum is initially unstable 
        self.body_angle_speed = 0

        self.robot_position = 0
        self.robot_speed = 0
        self.robot_acceleration = 0

    def f(self, body_angle, body_angle_speed, robot_position, robot_speed, deltat, tau):
        """ Function to evaluate the new state of the system:
            body_angle, body_angle_speed, robot_position, robot_speed, robot_acceleration
        """
        Mb = self.Mb
        Mw = self.Mw
        d  = self.d
        Ix = self.Ix
        Iz = self.Iz
        g  = self.g


        # mathematical model
        self.body_angle_acceleration = (tau - (Mb*d*g*sin(body_angle) + Ix*body_angle_speed*body_angle_speed*sin(body_angle)*cos(body_angle)))/(Iz - Ix*sin(body_angle)*sin(body_angle))
        self.robot_acceleration = (tau - (Mb*d*g*sin(body_angle) + Ix*body_angle_speed*body_angle_speed*sin(body_angle)*cos(body_angle)))/(Mw + Mb)
        if self.robot_acceleration > 0:
            self.robot_acceleration = min(self.robot_acceleration, 1)
        else:
            self.robot_acceleration = max(self.robot_acceleration, -1)
        '''
        if self.robot_acceleration > 0 and self.robot_acceleration < 0.05:
            self.robot_acceleration = 0.05
        if self.robot_acceleration < 0 and self.robot_acceleration > -0.05:
            self.robot_acceleration = -0.05
        '''
        if self.robot_speed >= 20:
            self.robot_acceleration = min(self.robot_acceleration, 0)

        self.body_angle = body_angle + body_angle_speed*deltat + self.body_angle_acceleration*deltat*deltat/2
        self.body_angle_speed = body_angle_speed + self.body_angle_acceleration*deltat
        self.robot_position = robot_position + robot_speed*deltat + self.robot_acceleration*deltat*deltat/2
        self.robot_speed = robot_speed + self.robot_acceleration*deltat
        if self.robot_speed > 0:
            self.robot_speed = min(self.robot_speed, 20)
        else:
            self.robot_speed = max(self.robot_speed, -20)

        return self.body_angle, self.body_angle_speed, self.robot_position, self.robot_speed, self.robot_acceleration

    def is_downed(self):
        if abs(self.body_angle) > pi/4:
            return True
        else:
            return False


    def dynamics(self, deltat, tau):
        """ Function to evaluate the new state of the system:
        """
        self.f(self.body_angle, self.body_angle_speed, self.robot_position, self.robot_speed, deltat, tau)

