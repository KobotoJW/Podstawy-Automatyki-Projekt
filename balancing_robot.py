from math import cos
from math import sin
from math import pi

class BalancingRobot:
    def __init__(self):
        # variables for the model
        self.Mb = None  # kg      mass of main body (pendulum)
        self.Mw = None  # kg      mass of wheels
        self.d = None   # m       center of mass from base
        self.Ix = None  # kg.m^2  Moment of inertia of body x-axis
        self.Iz = None  # kg.m^2  Moment of inertia of body z-axis
        self.g = None   # m/s^2   Acceleration due to gravity

        # variables for dynamic evaluation
        self.phi = None    # angle of the pendulum
        self.phip = None   # angular speed of the pendulum
        self.phipp = None  # angular acceleration of the pendulum
        self.x = None      # x position of the robot
        self.xp = None     # linear x speed of the robot
        self.xpp = None    # linear x acceleration of the robot

        self.initRobot()

    def initRobot(self):
        self.Mb = 13.3    # kg      mass of main body (pendulum)
        self.Mw = 1.5     # kg      mass of wheels
        self.d = 0.03     # m       center of mass from base
        self.Ix = 0.1935  # kg.m^2  Moment of inertia of body x-axis
        self.Iz = 0.3379  # kg.m^2  Moment of inertia of body z-axis
        self.g = 9.81     # m/s^2   Acceleration due to gravity  

        self.phi = 2*pi/180  # the pendulum is initially unstable 
        self.phip = 0

        self.x = 0
        self.xp = 0

    def f(self, phi, phip, x, xp, deltat, tau):
        """ Function to evaluate the new state of the system:
            phi, phip, x, xp
        """
        Mb = self.Mb
        Mw = self.Mw
        d  = self.d
        Ix = self.Ix
        Iz = self.Iz
        g  = self.g

        # mathematical model
        phipp = (tau - (Mb*d*g*sin(phi) + Ix*phip*phip*sin(phi)*cos(phi)))/(Iz - Ix*sin(phi)*sin(phi))
        xpp = (2*tau - (2*Mb*d*g*sin(phi) + 2*Ix*phip*phip*sin(phi)*cos(phi)))/(2*Mw + 2*Mb)
        
        self.phi = phi + phip*deltat + phipp*deltat*deltat/2
        self.phip = phip + phipp*deltat
        self.x = x + xp*deltat + xpp*deltat*deltat/2
        self.xp = xp + xpp*deltat
        return self.phi, self.phip, self.x, self.xp

    def is_downed(self):
        if abs(self.phi) > pi/4:
            return True
        else:
            return False


    def dynamics(self, deltat, tau):
        """ Function to evaluate the new state of the system:
        """
        self.f(self.phi, self.phip, self.x, self.xp, deltat, tau)

