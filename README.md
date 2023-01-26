# Podstawy-Automatyki-Projekt
This project is a simulation of a two-wheeled inverted pendulum robot using Python. The project includes a mathematical model of the robot's dynamics and a Proportional-Integral-Derivative (PID) controller for controlling its motion. The simulation plots the robot's angle and speed relative to the ground.

To run the simulation, you need to have Python 3 and the following package installed:
matplotlib;

The main file to run the simulation is "main.py". Before running the file, you can adjust the simulation parameters such as the duration of the simulation and the PID controller gains in the same file.

The simulation starts with an unstable initial condition where the pendulum is tilted at an angle of 2 degrees. The simulation runs for the specified duration and the robot's motion is updated using a Runge-Kutta approach to solve the differential equations of the model. The robot is able to balance itself and maintain a stable pendulum angle.

The robot's motion is controlled by the PID controller which takes the pendulum's angle as input and generates torque commands for the wheels as output. The controller's parameters (proportional gain, integral gain, and derivative gain) were adjusted to achieve stable performance.

The project includes a class called "BalancingRobot" which contains the mathematical model of the robot's dynamics and a class called "PID" which contains the implementation of the PID controller. The main function uses these classes to run the simulation and plot the results.
