"""Detect obstacle and stop controller."""
from controller import Robot, DistanceSensor

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

# Set wheel speeds (rad/s).
speed_left = 6.0
speed_right = 6.0

# Get proximity sensors (e-puck has 8: ps0..ps7).
sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
proximity_sensors = []
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    proximity_sensors.append(sensor)

# If any sensor reads above this value, stop.
obstacle_threshold = 80.0

# Run every simulation step.
while robot.step(timestep) != -1:
    obstacle_detected = False
    for sensor in proximity_sensors:
        if sensor.getValue() > obstacle_threshold:
            obstacle_detected = True
            break

    if obstacle_detected:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
    else:
        left_motor.setVelocity(speed_left)
        right_motor.setVelocity(speed_right)
