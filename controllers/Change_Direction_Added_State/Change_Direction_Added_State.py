from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

forward_time = 2.0
turn_left_time = 2
turn_right_time = 0.3

# Define states with explicit speeds
forward_state = {
    "left_speed": 3.0,
    "right_speed": 3.0,
    "duration": forward_time
}

turn_left_state = {
    "left_speed": -3.0,
    "right_speed": 3.0,
    "duration": turn_left_time
}

turn_right_state = {
    "left_speed": 3.0,
    "right_speed": -3.0,
    "duration": turn_right_time
}

# Control flow: forward -> left -> forward -> 
state_order = ["forward", "turn_left", "forward", "turn_right"]
states = {
    "forward": forward_state,
    "turn_left": turn_left_state,
    "turn_right": turn_right_state,
    
}

state_index = 0
time_in_state = 0.0

while robot.step(timestep) != -1:
    time_in_state += timestep / 1000.0
    
    state = state_order[state_index]
    current = states[state]
    left_motor.setVelocity(current["left_speed"])
    right_motor.setVelocity(current["right_speed"])
    
    if time_in_state >= current["duration"]:
        state_index = (state_index + 1) % len(state_order)
        time_in_state = 0.0
