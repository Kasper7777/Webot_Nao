from controller import Supervisor, Motion
import math
import os
import random
import json

# =============================
# RL CONFIG (easy defaults)
# =============================
MAX_EPISODES = 200
MAX_STEPS = 120
ALPHA = 0.2
GAMMA = 0.95
EPSILON = 0.1

DIST_BINS = [0.2, 0.4, 0.6, 1.0, 2.0]
ANGLE_BINS = [-0.5, -0.2, 0.2, 0.5]

ACTIONS = ["turn_left", "turn_right", "forward", "kick"]

# =============================
# WEBOTS SETUP
# =============================
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

nao_node = robot.getSelf()
duck_node = robot.getFromDef("DUCK")

if nao_node is None:
    nao_node = robot.getFromDef("NAO")

if nao_node is None or duck_node is None:
    print("Missing DEFs: ensure NAO is DEF NAO and RubberDuck is DEF DUCK in the world.")

nao_translation_field = nao_node.getField("translation")
nao_rotation_field = nao_node.getField("rotation")

init_nao_translation = list(nao_translation_field.getSFVec3f())
init_nao_rotation = list(nao_rotation_field.getSFRotation())
init_duck_translation = list(duck_node.getField("translation").getSFVec3f())
init_duck_rotation = list(duck_node.getField("rotation").getSFRotation())

# Enable cameras for vision-based learning
camera_top = robot.getDevice("CameraTop")
camera_bottom = robot.getDevice("CameraBottom")
if camera_top:
    camera_top.enable(timestep)
if camera_bottom:
    camera_bottom.enable(timestep)

WEBOTS_HOME = os.environ.get("WEBOTS_HOME", "/Applications/Webots.app/Contents")


def resolve_motion_dir(webots_home):
    if webots_home.endswith(".app"):
        webots_home = os.path.join(webots_home, "Contents")
    motion_dir = os.path.join(webots_home, "projects/robots/softbank/nao/motions")
    if os.path.isdir(motion_dir):
        return motion_dir
    if webots_home.endswith("/Contents"):
        alt_home = webots_home[: -len("/Contents")]
        alt_motion_dir = os.path.join(alt_home, "Contents/projects/robots/softbank/nao/motions")
        if os.path.isdir(alt_motion_dir):
            return alt_motion_dir
    return motion_dir


MOTION_DIR = resolve_motion_dir(WEBOTS_HOME)


def load_motion(name):
    path = os.path.join(MOTION_DIR, name)
    if not os.path.isfile(path):
        print(f"Motion missing: {path}")
        return None
    try:
        return Motion(path)
    except Exception as e:
        print(f"Motion load error: {e}")
        return None


forwards_motion = load_motion("Forwards50.motion")
turn_left_motion = load_motion("TurnLeft60.motion")
turn_right_motion = load_motion("TurnRight60.motion")
# KickRight doesn't exist, use TurnLeft as a placeholder kick motion
kick_right_motion = load_motion("TurnLeft60.motion")


def step_for(ms):
    steps = int(ms / timestep)
    for _ in range(max(1, steps)):
        if robot.step(timestep) == -1:
            return False
    return True


def play_motion(motion, max_steps=40):
    if motion is None:
        return step_for(640)
    motion.play()
    for _ in range(max_steps):
        if robot.step(timestep) == -1:
            return False
        if motion.isOver():
            return True
    motion.stop()
    return True


def reset_episode():
    nao_translation_field.setSFVec3f(init_nao_translation)
    nao_rotation_field.setSFRotation(init_nao_rotation)
    duck_node.getField("translation").setSFVec3f(init_duck_translation)
    duck_node.getField("rotation").setSFRotation(init_duck_rotation)
    robot.simulationResetPhysics()
    step_for(128)


# =============================
# STATE + REWARD
# =============================

def get_active_camera():
    """Choose camera based on head pitch."""
    if camera_top is None and camera_bottom is None:
        return None
    try:
        head_pitch = robot.getDevice("HeadPitch")
        if head_pitch:
            pitch = head_pitch.getTargetPosition()
            if pitch and pitch > 0.6:
                return camera_bottom
    except:
        pass
    return camera_top


def get_yellow_percentage():
    """Return yellow pixel percentage in current camera image (0.0 to 1.0)."""
    camera = get_active_camera()
    if not camera:
        return 0.0

    try:
        image = camera.getImage()
        if not image:
            return 0.0

        width = camera.getWidth()
        height = camera.getHeight()

        yellow_count = 0
        total_pixels = width * height

        for y in range(0, height, 4):
            for x in range(0, width, 4):
                r = camera.imageGetRed(image, width, x, y)
                g = camera.imageGetGreen(image, width, x, y)
                b = camera.imageGetBlue(image, width, x, y)

                # Yellow: high R, high G, low B
                if r > 150 and g > 150 and b < 100:
                    yellow_count += 1

        return yellow_count / (total_pixels / 16.0)
    except:
        return 0.0


def get_state():
    """Get state based on camera vision of yellow object."""
    yellow_pct = get_yellow_percentage()

    # Discretize yellow percentage into bins
    if yellow_pct < 0.01:
        yellow_bin = 0  # Not visible
    elif yellow_pct < 0.05:
        yellow_bin = 1  # Far
    elif yellow_pct < 0.15:
        yellow_bin = 2  # Medium
    else:
        yellow_bin = 3  # Close/large

    # Simple angle estimation: if yellow is centered, angle ~0
    # For now, use a placeholder (could add more camera processing for angle)
    angle_bin = 1

    return (yellow_bin, angle_bin, yellow_pct)


def reward_for(prev_yellow, new_yellow, kicked, duck_moved, duck_height):
    reward = -0.01
    # Reward for getting closer to yellow (increase in visibility)
    reward += 0.2 * (new_yellow - prev_yellow)
    # Reward for lifting it
    if duck_height > 0.02:
        reward += 3.0
    # Reward for kicking it far
    if kicked and duck_moved > 0.2:
        reward += 5.0
    return reward


# =============================
# Q-TABLE UTILS
# =============================

Q_PATH = os.path.join(os.path.dirname(__file__), "q_table.json")


def load_q():
    if os.path.isfile(Q_PATH):
        try:
            with open(Q_PATH, "r", encoding="utf-8") as f:
                raw = json.load(f)
            return {tuple(map(int, k.split(","))): v for k, v in raw.items()}
        except Exception:
            return {}
    return {}


def save_q(q_table):
    serial = {f"{k[0]},{k[1]}": v for k, v in q_table.items()}
    with open(Q_PATH, "w", encoding="utf-8") as f:
        json.dump(serial, f, indent=2)


def q_values(q_table, state):
    if state not in q_table:
        q_table[state] = [0.0 for _ in ACTIONS]
    return q_table[state]


# =============================
# TRAINING LOOP
# =============================

q_table = load_q()

for episode in range(MAX_EPISODES):
    reset_episode()

    duck_start = list(duck_node.getField("translation").getSFVec3f())
    total_reward = 0.0

    for step in range(MAX_STEPS):
        state = get_state()
        prev_yellow = state[2]

        if random.random() < EPSILON:
            action_idx = random.randrange(len(ACTIONS))
        else:
            qv = q_values(q_table, state[:2])
            action_idx = int(max(range(len(qv)), key=lambda i: qv[i]))

        action = ACTIONS[action_idx]

        if action == "turn_left":
            play_motion(turn_left_motion)
        elif action == "turn_right":
            play_motion(turn_right_motion)
        elif action == "forward":
            play_motion(forwards_motion)
        elif action == "kick":
            play_motion(kick_right_motion)

        new_state = get_state()
        new_yellow = new_state[2]

        duck_now = list(duck_node.getField("translation").getSFVec3f())
        duck_moved = math.sqrt(
            (duck_now[0] - duck_start[0]) ** 2 + (duck_now[2] - duck_start[2]) ** 2
        )

        duck_height = duck_now[1] - duck_start[1]

        reward = reward_for(prev_yellow, new_yellow, action == "kick", duck_moved, duck_height)
        total_reward += reward

        qv = q_values(q_table, state[:2])
        qv_next = q_values(q_table, new_state[:2])
        qv[action_idx] = qv[action_idx] + ALPHA * (
            reward + GAMMA * max(qv_next) - qv[action_idx]
        )

        if duck_moved > 0.3:
            break

    if (episode + 1) % 10 == 0:
        save_q(q_table)
        print(f"Episode {episode + 1} reward={total_reward:.2f}")

save_q(q_table)
print("Training complete. Q-table saved.")

# Keep stepping so Webots doesn't exit immediately
while robot.step(timestep) != -1:
    pass
