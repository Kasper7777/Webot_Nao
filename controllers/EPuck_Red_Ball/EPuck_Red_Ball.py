"""
E-puck robot that finds and moves toward a red ball.
Uses camera vision to detect red color and proportional control to steer.
"""

from controller import Robot
import math

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Get motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Get camera
camera = robot.getDevice("camera")
camera.enable(timestep)

# Get distance sensors (optional, for obstacle avoidance)
sensors = [robot.getDevice(f"ps{i}") for i in range(8)]
for sensor in sensors:
    sensor.enable(timestep)

MAX_SPEED = 6.28  # rad/s

def get_red_position():
    """
    Detect red ball in camera image.
    Returns (red_count, center_x) or (0, -1) if no red found.
    """
    image = camera.getImage()
    if not image:
        return 0, -1
    
    width = camera.getWidth()
    height = camera.getHeight()
    
    red_count = 0
    red_x_sum = 0
    
    # Scan every 2 pixels for speed
    for y in range(0, height, 2):
        for x in range(0, width, 2):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)
            
            # Red: high R, low G and B
            if r > 200 and g < 100 and b < 100:
                red_count += 1
                red_x_sum += x
    
    if red_count == 0:
        return 0, -1
    
    center_x = red_x_sum / red_count
    return red_count, center_x


def avoid_obstacles():
    """Check front sensors for obstacles."""
    front_sensors = [sensors[7].getValue(), sensors[0].getValue(), sensors[1].getValue()]
    return max(front_sensors) > 80  # Threshold for obstacle


def main():
    print("E-puck Red Ball Tracker Started")
    print("Looking for red ball...")
    
    while robot.step(timestep) != -1:
        red_count, red_x = get_red_position()
        
        width = camera.getWidth()
        center = width / 2
        
        if red_count == 0:
            # No red ball visible - spin slowly to search
            print(f"No red ball found. Spinning...")
            left_motor.setVelocity(MAX_SPEED * 0.3)
            right_motor.setVelocity(-MAX_SPEED * 0.3)
        else:
            # Red ball found - use proportional control to aim at it
            error = red_x - center  # Negative = red is left, Positive = red is right
            
            # Proportional steering
            steering = error / center  # Normalized error [-1, 1]
            steering = max(-1, min(1, steering))  # Clamp
            
            # Move forward with steering
            base_speed = MAX_SPEED * 0.8
            left_speed = base_speed * (1 + steering * 0.5)
            right_speed = base_speed * (1 - steering * 0.5)
            
            # Check for obstacles
            if avoid_obstacles():
                print("Obstacle detected! Backing up...")
                left_motor.setVelocity(-base_speed * 0.5)
                right_motor.setVelocity(-base_speed * 0.5)
            else:
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                
                print(f"Red: {red_count} pixels at x={red_x:.0f}, steering={steering:+.2f}")

if __name__ == "__main__":
    main()
