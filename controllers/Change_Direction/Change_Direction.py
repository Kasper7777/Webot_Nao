from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

forward_time = 3.0
turn_left_time = 0.5
reverse_time = 4.0
spin_time = 6.0

# Define states with explicit speeds
forward_state = {
    "left_speed": 3.0,
    "right_speed": 3.0,
    "duration": forward_time
}

turn_left_state = {
    "left_speed": -6.0,
    "right_speed": 6.0,
    "duration": turn_left_time
}

reverse_state = {
    "left_speed": -6.0,
    "right_speed": -6.0,
    "duration": reverse_time
}

spin_state = {
    "left_speed": 6.0,
    "right_speed": -6.0,
    "duration": spin_time
}
# Control flow: forward -> left -> forward -> right...
state_order = ["forward", "turn_left", "forward", "reverse", "spin"]
states = {
    "forward": forward_state,
    "turn_left": turn_left_state,
    "reverse" : reverse_state,
    "spin" : spin_state
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
