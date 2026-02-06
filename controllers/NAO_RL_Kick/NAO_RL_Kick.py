from controller import Supervisor, Motion
import math
import os
import random
import json
import time

# =============================
# RL CONFIG - MOTION-BASED
# =============================
MAX_EPISODES = 100
MAX_STEPS = 100
MAX_TIME_PER_EPISODE = 20.0  # 20 seconds per episode
ALPHA = 0.15
GAMMA = 0.95
EPSILON = 0.2

# Actions: motion files + bilateral arm control
ACTIONS = [
    "turn_left", 
    "turn_right", 
    "forward", 
    "side_step_left",
    "reach_forward_both",
    "reach_down_both",
    "close_hands",
    "open_hands"
]

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

# ===== HEAD MOTORS =====
head_yaw = robot.getDevice("HeadYaw")
head_pitch = robot.getDevice("HeadPitch")

# ===== RIGHT ARM MOTORS =====
r_shoulder_pitch = robot.getDevice("RShoulderPitch")
r_shoulder_roll = robot.getDevice("RShoulderRoll")
r_elbow_roll = robot.getDevice("RElbowRoll")
r_elbow_yaw = robot.getDevice("RElbowYaw")
r_wrist_yaw = robot.getDevice("RWristYaw")

# ===== LEFT ARM MOTORS =====
l_shoulder_pitch = robot.getDevice("LShoulderPitch")
l_shoulder_roll = robot.getDevice("LShoulderRoll")
l_elbow_roll = robot.getDevice("LElbowRoll")
l_elbow_yaw = robot.getDevice("LElbowYaw")
l_wrist_yaw = robot.getDevice("LWristYaw")

# ===== LEG MOTORS =====
l_hip_yaw_pitch = robot.getDevice("LHipYawPitch")
l_hip_roll = robot.getDevice("LHipRoll")
l_hip_pitch = robot.getDevice("LHipPitch")
l_knee_pitch = robot.getDevice("LKneePitch")
l_ankle_pitch = robot.getDevice("LAnklePitch")
l_ankle_roll = robot.getDevice("LAnkleRoll")

r_hip_yaw_pitch = robot.getDevice("RHipYawPitch")
r_hip_roll = robot.getDevice("RHipRoll")
r_hip_pitch = robot.getDevice("RHipPitch")
r_knee_pitch = robot.getDevice("RKneePitch")
r_ankle_pitch = robot.getDevice("RAnklePitch")
r_ankle_roll = robot.getDevice("RAnkleRoll")

# Enable keyboard input
keyboard = robot.getDevice("keyboard")
if keyboard:
    keyboard.enable(timestep)
else:
    keyboard = None

# Try to enable display overlay (for on-screen stats)
display = None
try:
    display = robot.getDevice("Display")
    if display:
        print(f"✓ Display found! Dimensions: {display.getWidth()}x{display.getHeight()}")
        display.setFont("Arial", 12, False)
    else:
        print("✗ Display device not found in world")
except Exception as e:
    print(f"✗ Display error: {e}")

# Load motion files
WEBOTS_HOME = os.environ.get("WEBOTS_HOME", "/Applications/Webots.app/Contents")

def resolve_motion_dir(webots_home):
    if webots_home.endswith(".app"):
        webots_home = os.path.join(webots_home, "Contents")
    motion_dir = os.path.join(webots_home, "projects/robots/softbank/nao/motions")
    if os.path.isdir(motion_dir):
        return motion_dir
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
side_step_left_motion = load_motion("SideStepLeft.motion")

# Arm motors for reaching and grabbing
r_shoulder_pitch = robot.getDevice("RShoulderPitch")
r_shoulder_roll = robot.getDevice("RShoulderRoll")
r_elbow_roll = robot.getDevice("RElbowRoll")
r_elbow_yaw = robot.getDevice("RElbowYaw")
r_wrist_yaw = robot.getDevice("RWristYaw")


def step_for(ms):
    """Step simulation for specified milliseconds."""
    steps = int(ms / timestep)
    for _ in range(max(1, steps)):
        if robot.step(timestep) == -1:
            return False
    return True


def play_motion(motion, max_steps=40):
    """Play a motion file."""
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
    """Reset robot and duck to initial state."""
    nao_translation_field.setSFVec3f(init_nao_translation)
    nao_rotation_field.setSFRotation(init_nao_rotation)
    duck_node.getField("translation").setSFVec3f(init_duck_translation)
    duck_node.getField("rotation").setSFRotation(init_duck_rotation)
    
    robot.simulationResetPhysics()
    step_for(128)


def execute_action(action):
    """Execute motion-based action."""
    if action == "turn_left":
        play_motion(turn_left_motion)
    elif action == "turn_right":
        play_motion(turn_right_motion)
    elif action == "forward":
        play_motion(forwards_motion)
    elif action == "side_step_left":
        play_motion(side_step_left_motion)
    
    # Arm actions - direct motor control
    # Bilateral arm actions (both arms together)
    elif action == "reach_forward_both":
        # Both arms forward to embrace duck
        if r_shoulder_pitch:
            r_shoulder_pitch.setPosition(0.5)
        if l_shoulder_pitch:
            l_shoulder_pitch.setPosition(0.5)
        if r_elbow_roll:
            r_elbow_roll.setPosition(0.8)
        if l_elbow_roll:
            l_elbow_roll.setPosition(-0.8)  # Mirror
        step_for(500)
    elif action == "reach_down_both":
        # Both arms down to reach duck at feet
        if r_shoulder_pitch:
            r_shoulder_pitch.setPosition(1.5)
        if l_shoulder_pitch:
            l_shoulder_pitch.setPosition(1.5)
        if r_elbow_roll:
            r_elbow_roll.setPosition(0.1)
        if l_elbow_roll:
            l_elbow_roll.setPosition(-0.1)  # Mirror
        step_for(500)
    elif action == "close_hands":
        # Both hands close for strong grip
        if r_wrist_yaw:
            r_wrist_yaw.setPosition(1.5)
        if l_wrist_yaw:
            l_wrist_yaw.setPosition(-1.5)  # Mirror
        step_for(400)
    elif action == "open_hands":
        # Both hands open to release
        if r_wrist_yaw:
            r_wrist_yaw.setPosition(-1.5)
        if l_wrist_yaw:
            l_wrist_yaw.setPosition(1.5)  # Mirror
        step_for(400)


# =============================
# STATE + REWARD
# =============================

def get_active_camera():
    """Choose camera based on head pitch."""
    if camera_top is None and camera_bottom is None:
        print("WARNING: No cameras found!")
        return None
    # For now, always use camera_top since it's available
    if camera_top:
        return camera_top
    return camera_bottom


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

                # ONLY match PURE yellow: R and G high, B VERY low
                if r > 180 and g > 150 and b < 50:
                    yellow_count += 1

        yellow_pct = yellow_count / (total_pixels / 16.0)
        return yellow_pct
    except Exception as e:
        print(f"Camera error: {e}")
        return 0.0


def debug_camera_sample():
    """Sample and print pixel colors from center of image."""
    camera = get_active_camera()
    if not camera:
        return
    try:
        image = camera.getImage()
        if not image:
            print("DEBUG: No image data")
            return
        width = camera.getWidth()
        height = camera.getHeight()
        
        # Sample center and corners
        samples = [
            ("center", width//2, height//2),
            ("top-left", width//4, height//4),
            ("top-right", 3*width//4, height//4),
        ]
        
        print(f"DEBUG: Camera image {width}x{height}")
        for name, x, y in samples:
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)
            print(f"  {name} ({x},{y}): R={r} G={g} B={b}")
    except Exception as e:
        print(f"DEBUG: Camera sample error: {e}")


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


def reward_for(duck_height, time_on_ground, manual_bonus=0):
    """
    Calculate reward based on duck height and time on ground.
    - Reward proportional to how high duck is lifted (exponential)
    - Penalty for time duck stays on ground (every 10 seconds = -1 point)
    - Manual bonus from keyboard input
    """
    reward = 0.0
    
    # Height reward: exponential - higher = much better!
    if duck_height > 0.001:  # Duck lifted at all
        # Exponential: small lift = small reward, big lift = huge reward
        reward += duck_height * 100  # Scale up
    
    # Time penalty: -0.1 points per second on ground (= -1 per 10 seconds)
    if duck_height < 0.001:
        reward -= time_on_ground * 0.1
    
    # Manual keyboard bonus
    reward += manual_bonus
    
    return reward


# =============================
# Q-TABLE UTILS
# =============================

Q_PATH = os.path.join(os.path.dirname(__file__), "q_table.json")
EPISODE_PATH = os.path.join(os.path.dirname(__file__), "episode.json")
Q_PATH_ALT = os.path.expanduser("~/Documents/Webot/controllers/NAO_RL_Kick/q_table.json")
EPISODE_PATH_ALT = os.path.expanduser("~/Documents/Webot/controllers/NAO_RL_Kick/episode.json")

print(f"Q-table path: {Q_PATH}")
print(f"Episode path: {EPISODE_PATH}")
print(f"Directory writable: {os.access(os.path.dirname(Q_PATH), os.W_OK)}")


def load_q():
    for path in [Q_PATH, Q_PATH_ALT]:
        if os.path.isfile(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    raw = json.load(f)
                print(f"✓ Loaded Q-table from {path} with {len(raw)} states")
                return {tuple(map(int, k.split(","))): v for k, v in raw.items()}
            except Exception as e:
                print(f"Error loading {path}: {e}")
    print("No existing Q-table found, starting fresh")
    return {}


def save_q(q_table):
    for path in [Q_PATH, Q_PATH_ALT]:
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            serial = {f"{k[0]},{k[1]}": v for k, v in q_table.items()}
            with open(path, "w", encoding="utf-8") as f:
                json.dump(serial, f, indent=2)
            print(f"✓ Saved Q-table with {len(q_table)} states")
            return
        except Exception as e:
            print(f"✗ Could not save to {path}: {e}")
    print("✗ FAILED to save Q-table!")


def load_episode():
    """Load current episode counter."""
    for path in [EPISODE_PATH, EPISODE_PATH_ALT]:
        if os.path.isfile(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                episode = data.get("episode", 0)
                print(f"✓ Resuming from episode {episode + 1}")
                return episode
            except:
                pass
    print("Starting training from episode 1")
    return 0


def save_episode(episode_num):
    """Save current episode counter."""
    for path in [EPISODE_PATH, EPISODE_PATH_ALT]:
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"episode": episode_num}, f)
            return
        except:
            pass


def q_values(q_table, state):
    if state not in q_table:
        q_table[state] = [0.0 for _ in ACTIONS]
    return q_table[state]


def check_manual_score():
    """Check for number key presses: 0-9 adds points, Shift+0-9 subtracts."""
    try:
        key = keyboard.getKey()
    except:
        return 0.0
    
    if key < 0:
        return 0.0
    
    # Check if shift is held
    is_shift = (key & Keyboard.SHIFT) != 0
    
    # Extract the actual key code
    base_key = key & 0xFF
    
    # Number keys: 0-9
    if ord('0') <= base_key <= ord('9'):
        value = float(chr(base_key))
        score = -value if is_shift else value
        print(f"🎹 KEYBOARD INPUT DETECTED: {chr(base_key)} (Shift={is_shift}) -> Score: {score:+.0f}")
        return score
    
    return 0.0


def show_stats(episode_num, step_num, total_reward, current_action, yellow_pct):
    """Display stats on screen and console."""
    if display:
        try:
            display.fillRectangle(0, 0, display.getWidth(), 160)  # Clear top area
            display.setColor(0xFFFF00)  # Yellow text
            display.drawText(f"Ep {episode_num} | Step {step_num}", 10, 10)
            display.drawText(f"Yellow: {yellow_pct:.1f}%", 10, 25)
            display.drawText(f"Action: {current_action}", 10, 40)
            display.setColor(0x00FF00 if total_reward >= 0 else 0xFF0000)  # Green if positive, red if negative
            display.drawText(f"REWARD: {total_reward:+.1f}", 10, 60)
            display.setColor(0xFFFFFF)  # White
            display.drawText("[0-9=reward] [Shift+0-9=penalize]", 10, 80)
        except Exception as e:
            print(f"Display draw error: {e}")


q_table = load_q()
start_episode = load_episode()
print(f"Starting training with {len(q_table)} existing states...")
print(f"Display enabled: {display is not None}")
print(f"Keyboard enabled: True")
print(f"CameraTop: {camera_top}")
print(f"CameraBottom: {camera_bottom}")
if camera_top:
    print(f"  CameraTop resolution: {camera_top.getWidth()}x{camera_top.getHeight()}")
if camera_bottom:
    print(f"  CameraBottom resolution: {camera_bottom.getWidth()}x{camera_bottom.getHeight()}")

# Debug first frame
print("Sampling camera pixels on startup...")
debug_camera_sample()
print(f"Starting training with {len(q_table)} existing states...\n")

try:
    for episode in range(start_episode, MAX_EPISODES):
        print(f"\n=== Episode {episode + 1}/{MAX_EPISODES} ===")
        reset_episode()
        
        duck_start_height = init_duck_translation[1]
        total_reward = 0.0
        time_on_ground = 0.0  # Track time duck is on ground
        episode_start_time = robot.getTime()  # Track episode duration

        for step in range(MAX_STEPS):
            # Check time limit
            elapsed_time = robot.getTime() - episode_start_time
            time_remaining = MAX_TIME_PER_EPISODE - elapsed_time
            
            if elapsed_time >= MAX_TIME_PER_EPISODE:
                print(f"  ⏱ TIME'S UP! (20 seconds elapsed)")
                break
            
            state = get_state()
            prev_yellow = state[2]

            # Epsilon-greedy action selection
            if random.random() < EPSILON:
                action_idx = random.randrange(len(ACTIONS))
            else:
                qv = q_values(q_table, state[:2])
                action_idx = int(max(range(len(qv)), key=lambda i: qv[i]))

            action = ACTIONS[action_idx]
            
            # Show what robot is doing
            print(f"  Step {step + 1} [{time_remaining:.1f}s left]: action={action}")
            
            execute_action(action)

            # Get new state after action
            new_state = get_state()
            new_yellow = new_state[2]

            # Calculate duck height above ground
            duck_now = list(duck_node.getField("translation").getSFVec3f())
            duck_height = duck_now[1] - duck_start_height
            
            # Track time on ground
            action_duration = robot.getTime() - (episode_start_time + elapsed_time)
            if duck_height < 0.001:
                time_on_ground += action_duration
            else:
                time_on_ground = 0  # Reset if lifted
            
            # Check for manual score
            manual_score = check_manual_score() if keyboard else 0.0

            # Calculate reward based on HEIGHT and TIME
            reward = reward_for(duck_height, time_on_ground, manual_score)
            total_reward += reward
            
            print(f"    Duck height: {duck_height:.3f}m | Time on ground: {time_on_ground:.1f}s | Reward: {reward:+.2f}")

            # Q-learning update
            qv = q_values(q_table, state[:2])
            qv_next = q_values(q_table, new_state[:2])
            qv[action_idx] = qv[action_idx] + ALPHA * (
                reward + GAMMA * max(qv_next) - qv[action_idx]
            )

            # Stop if duck lifted very high (success!)
            if duck_height > 0.15:
                print(f"  ✓ LIFTED DUCK HIGH! ({duck_height:.3f}m)")
                break
        
        print(f"Episode {episode + 1} total_reward={total_reward:.2f}")
        
        # Save after every episode
        save_q(q_table)
        save_episode(episode + 1)

    print("\n✓✓✓ All episodes complete!")

except Exception as e:
    print(f"\n✗✗✗ CRASH: {e}")
    import traceback
    traceback.print_exc()
    save_q(q_table)
    save_episode(episode + 1)

# Keep stepping
while robot.step(timestep) != -1:
    pass
