"""Move forward controller."""
from controller import Robot

# Create the robot object.
robot = Robot()

# Get the simulation time step.
timestep = int(robot.getBasicTimeStep())

# Get the two wheel motors by name.
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Switch motors to velocity control mode.
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

# Set wheel speeds (rad/s). Use different values to turn.
speed_left = 6.0
speed_right = 0.0

# Run every simulation step and keep the wheels spinning.
while robot.step(timestep) != -1:
    left_motor.setVelocity(speed_left)
    right_motor.setVelocity(speed_right)
