from controller import Supervisor, Keyboard
import math
import os
import random
import json

# =============================
# RL CONFIG - FULL MOTOR CONTROL
# =============================
MAX_EPISODES = 100
MAX_STEPS = 100
ALPHA = 0.1
GAMMA = 0.95
EPSILON = 0.15

# Actions: head control, arm control, locomotion, grabbing
ACTIONS = [
    "head_left",
    "head_right",
    "head_center",
    "arm_forward",
    "arm_down",
    "arm_up",
    "walk_forward",
    "walk_left",
    "walk_right",
    "walk_backward",
    "grab",
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
keyboard = Keyboard()
keyboard.enable(timestep)

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


def step_for(ms):
    """Step simulation for specified milliseconds."""
    steps = int(ms / timestep)
    for _ in range(max(1, steps)):
        if robot.step(timestep) == -1:
            return False
    return True


def reset_episode():
    """Reset robot and duck to initial state."""
    nao_translation_field.setSFVec3f(init_nao_translation)
    nao_rotation_field.setSFRotation(init_nao_rotation)
    duck_node.getField("translation").setSFVec3f(init_duck_translation)
    duck_node.getField("rotation").setSFRotation(init_duck_rotation)
    
    # Reset all motors to neutral positions
    if head_yaw:
        head_yaw.setPosition(0)
    if head_pitch:
        head_pitch.setPosition(0)
    if r_shoulder_pitch:
        r_shoulder_pitch.setPosition(1.5)  # Down
    if r_shoulder_roll:
        r_shoulder_roll.setPosition(-0.3)  # Towards body
    if r_elbow_roll:
        r_elbow_roll.setPosition(0.1)  # Valid range: [0, 1.54]
    if r_elbow_yaw:
        r_elbow_yaw.setPosition(0)
    if r_wrist_yaw:
        r_wrist_yaw.setPosition(0)
    
    # Reset legs to standing
    for leg_motor in [l_hip_pitch, r_hip_pitch, l_knee_pitch, r_knee_pitch, l_ankle_pitch, r_ankle_pitch]:
        if leg_motor:
            leg_motor.setPosition(0)
    
    robot.simulationResetPhysics()
    step_for(128)


def execute_action(action):
    """Execute motor command based on RL action."""
    if action == "head_left":
        if head_yaw:
            head_yaw.setPosition(0.8)
        step_for(200)
    elif action == "head_right":
        if head_yaw:
            head_yaw.setPosition(-0.8)
        step_for(200)
    elif action == "head_center":
        if head_yaw:
            head_yaw.setPosition(0)
        step_for(200)
    
    elif action == "arm_forward":
        if r_shoulder_pitch:
            r_shoulder_pitch.setPosition(0.8)  # Forward
        if r_elbow_roll:
            r_elbow_roll.setPosition(0.8)  # Extend (valid: 0-1.54)
        step_for(400)
    elif action == "arm_down":
        if r_shoulder_pitch:
            r_shoulder_pitch.setPosition(1.5)  # Down
        if r_elbow_roll:
            r_elbow_roll.setPosition(0.1)  # Retract (valid: 0-1.54)
        step_for(400)
    elif action == "arm_up":
        if r_shoulder_pitch:
            r_shoulder_pitch.setPosition(0.2)  # Up
        if r_elbow_roll:
            r_elbow_roll.setPosition(0.5)  # Extend (valid: 0-1.54)
        step_for(400)
    
    elif action == "walk_forward":
        if l_hip_pitch:
            l_hip_pitch.setPosition(-0.5)
        if r_hip_pitch:
            r_hip_pitch.setPosition(-0.5)
        step_for(500)
    elif action == "walk_left":
        if l_hip_roll:
            l_hip_roll.setPosition(0.3)
        if r_hip_roll:
            r_hip_roll.setPosition(-0.3)
        step_for(500)
    elif action == "walk_right":
        if l_hip_roll:
            l_hip_roll.setPosition(-0.3)
        if r_hip_roll:
            r_hip_roll.setPosition(0.3)
        step_for(500)
    elif action == "walk_backward":
        if l_hip_pitch:
            l_hip_pitch.setPosition(0.3)  # Clamped from 0.5 (max 0.48398)
        if r_hip_pitch:
            r_hip_pitch.setPosition(0.3)  # Clamped from 0.5 (max 0.48398)
        step_for(500)
    
    elif action == "grab":
        if r_wrist_yaw:
            r_wrist_yaw.setPosition(1.5)  # Close fingers
        step_for(300)


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


def reward_for(prev_yellow, new_yellow, action, duck_moved, duck_height):
    """Calculate reward for this action."""
    reward = -0.02  # Time penalty
    # Reward for approaching yellow (increase in visibility)
    reward += 0.3 * (new_yellow - prev_yellow)
    # Reward for lifting it
    if duck_height > 0.05:
        reward += 4.0
    # Reward for successfully grabbing and moving
    if action == "grab" and duck_moved > 0.1:
        reward += 6.0
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
        
        # Debug: Sample camera on first step of first episode
        if episode == start_episode:
            step_for(100)
            debug_camera_sample()

        duck_start = list(duck_node.getField("translation").getSFVec3f())
        total_reward = 0.0
        episode_actions = []

        for step in range(MAX_STEPS):
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
            print(f"  Step {step + 1}: yellow={prev_yellow:.2f}% -> action={action}")
            print(f"    [Press 0-9 to reward] [Shift+0-9 to penalize]")
            
            execute_action(action)

            # Get new state after action
            new_state = get_state()
            new_yellow = new_state[2]

            # Calculate reward
            duck_now = list(duck_node.getField("translation").getSFVec3f())
            duck_moved = math.sqrt(
                (duck_now[0] - duck_start[0]) ** 2 + (duck_now[2] - duck_start[2]) ** 2
            )
            duck_height = duck_now[1] - duck_start[1]

            reward = reward_for(prev_yellow, new_yellow, action, duck_moved, duck_height)
            
            # Check for manual scoring
            manual_score = check_manual_score()
            if manual_score != 0.0:
                reward += manual_score
                sign = "+" if manual_score > 0 else "-"
                print(f"    {sign} MANUAL SCORE: {abs(manual_score):.0f} (total reward now: {reward:.2f})")
            
            total_reward += reward
            
            episode_actions.append({
                "step": step,
                "action": action,
                "yellow_before": prev_yellow,
                "yellow_after": new_yellow,
                "duck_height": duck_height,
                "reward": reward
            })

            # Q-learning update
            qv = q_values(q_table, state[:2])
            qv_next = q_values(q_table, new_state[:2])
            qv[action_idx] = qv[action_idx] + ALPHA * (
                reward + GAMMA * max(qv_next) - qv[action_idx]
            )

            # Show stats on screen
            show_stats(episode + 1, step + 1, total_reward, action, prev_yellow)

            # Stop if duck moved significantly (success)
            if duck_moved > 0.5:
                print(f"  ✓ MOVED DUCK! ({duck_moved:.2f}m)")
                break
        
        print(f"Episode {episode + 1} total_reward={total_reward:.2f}")
        
        # MANUAL SCORING: Press SPACEBAR during episode for +5 bonus
        print("  [PRESS SPACEBAR to manually reward robot +5 points]")
        
        # Save after every episode now
        save_q(q_table)
        save_episode(episode + 1)

    print("\n✓✓✓ All episodes complete!")

except Exception as e:
    print(f"\n✗✗✗ CRASH: {e}")
    import traceback
    traceback.print_exc()
    save_q(q_table)
    save_episode(episode + 1)

# Keep stepping so Webots doesn't exit immediately
while robot.step(timestep) != -1:
    pass
