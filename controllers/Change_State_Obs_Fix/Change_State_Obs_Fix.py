from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
proximity_sensor = robot.getDevice("ps0")
proximity_sensor.enable(timestep)

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

obstacle_threshold = 80.0

# Define states
forward_state = {"left_speed": 6.0, "right_speed": 6.0}
turn_left_state = {"left_speed": -6.0, "right_speed": 6.0}
turn_right_state = {"left_speed": 6.0, "right_speed": -6.0}
spin_right_state = {"left_speed": 6.0, "right_speed": -6.0}

states = {"forward": forward_state, "turn_left": turn_left_state, "turn_right": turn_right_state, "spin_right": spin_right_state}

state = "forward"
stuck_counter = 0
time_in_state = 0.0
min_turn_time = 3

while robot.step(timestep) != -1:
    time_in_state += timestep / 1000.0
    sensor_value = proximity_sensor.getValue()
    left_motor.setVelocity(states[state]["left_speed"])
    right_motor.setVelocity(states[state]["right_speed"])
    
    if state == "forward":
        if sensor_value > obstacle_threshold:
            state = "turn_left"
            stuck_counter += 1
            time_in_state = 0.0
    elif state == "turn_left":
        if time_in_state >= min_turn_time and sensor_value < obstacle_threshold:
            state = "forward"
            time_in_state = 0.0
    elif state == "turn_right":
        if time_in_state >= min_turn_time and sensor_value < obstacle_threshold:
            state = "forward"
            time_in_state = 0.0
    elif state == "spin_right":
        if sensor_value < obstacle_threshold:
            state = "turn_left"
            stuck_counter = 0
            time_in_state = 0.0
    
    # If stuck too many times, spin right
    if stuck_counter >= 2 and state != "spin_right":
        state = "spin_right"
        stuck_counter = 0
        time_in_state = 0.0
    # Alternate turns on next obstacle
    elif state == "turn_left" and stuck_counter % 2 == 0:
        if time_in_state >= min_turn_time * 2:
            state = "turn_right"
            time_in_state = 0.0
