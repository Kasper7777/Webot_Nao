from controller import Robot, Motion
import math
import os
from datetime import datetime

# ============================================================================
# CONSTANTS - Easy to modify
# ============================================================================
MOVEMENT_SPEED = 5.0  # Forward movement speed
TURN_SPEED = 2.0      # Turning speed
YELLOW_COLOR_THRESHOLD = 50  # Adjust for yellow detection sensitivity
YELLOW_DETECT_PERCENT = 0.003  # 0.3% yellow pixels to detect object
YELLOW_NEAR_PERCENT = 0.035     # 3.5% yellow pixels means close enough to pick up
HEAD_TRACK_YAW_GAIN = 2.0
HEAD_TRACK_PITCH_GAIN = 1.4
HEAD_TRACK_SMOOTHING = 0.35  # 0..1, higher = smoother
HEAD_TRACK_MAX_STEP = 0.06   # max rad per timestep
APPROACH_TILT_TARGET = 0.25  # ~14 degrees in radians
PICKUP_STAGE_TIME = 2400     # ms per pickup stage
FALL_ANGLE_THRESHOLD = 0.7    # radians (~40 degrees)
DISTANCE_SCALE = 0.7          # Tunable scale for distance estimate (meters)
DISTANCE_MIN = 0.15           # Minimum distance clamp (meters)
DISTANCE_MAX = 2.5            # Maximum distance clamp (meters)
DISTANCE_TO_OBJECT = 0.5   # Distance threshold for approaching object
WEBOTS_HOME = os.environ.get("WEBOTS_HOME", "/Applications/Webots.app/Contents")

def resolve_motion_dir(webots_home):
    """Resolve the NAO motions directory from WEBOTS_HOME."""
    # If WEBOTS_HOME points to the app root, add /Contents
    if webots_home.endswith(".app"):
        webots_home = os.path.join(webots_home, "Contents")
    motion_dir = os.path.join(webots_home, "projects/robots/softbank/nao/motions")
    if os.path.isdir(motion_dir):
        return motion_dir
    # Fallback: try app root if given a Contents path already
    if webots_home.endswith("/Contents"):
        alt_home = webots_home[: -len("/Contents")]
        alt_motion_dir = os.path.join(alt_home, "Contents/projects/robots/softbank/nao/motions")
        if os.path.isdir(alt_motion_dir):
            return alt_motion_dir
    return motion_dir

MOTION_DIR = resolve_motion_dir(WEBOTS_HOME)
if not os.path.isdir(MOTION_DIR):
    MOTION_DIR = "/Applications/Webots.app/Contents/projects/robots/softbank/nao/motions"

# ============================================================================
# ROBOT INITIALIZATION
# ============================================================================
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Head joints for looking around
head_yaw = robot.getDevice("HeadYaw")      # Look left/right
head_pitch = robot.getDevice("HeadPitch")  # Look up/down

# Right arm for picking up
r_shoulder_pitch = robot.getDevice("RShoulderPitch")
r_shoulder_roll = robot.getDevice("RShoulderRoll")
r_elbow_roll = robot.getDevice("RElbowRoll")
r_elbow_yaw = robot.getDevice("RElbowYaw")
r_wrist_yaw = robot.getDevice("RWristYaw")

# Left arm
l_shoulder_pitch = robot.getDevice("LShoulderPitch")
l_shoulder_roll = robot.getDevice("LShoulderRoll")
l_elbow_roll = robot.getDevice("LElbowRoll")
l_elbow_yaw = robot.getDevice("LElbowYaw")
l_wrist_yaw = robot.getDevice("LWristYaw")

# Leg motors for walking
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

# Camera for object detection
camera_top = robot.getDevice("CameraTop")
camera_bottom = robot.getDevice("CameraBottom")
camera_top.enable(timestep)
camera_bottom.enable(timestep)

# Inertial unit for fall detection
inertial_unit = robot.getDevice("inertial unit")
inertial_unit.enable(timestep)

# Foot sensors (force sensors and bumpers)
lf_fsr = robot.getDevice("LFsr")
rf_fsr = robot.getDevice("RFsr")
lf_fsr.enable(timestep)
rf_fsr.enable(timestep)

lfootlbumper = robot.getDevice("LFoot/Bumper/Left")
lfootrbumper = robot.getDevice("LFoot/Bumper/Right")
rfootlbumper = robot.getDevice("RFoot/Bumper/Left")
rfootrbumper = robot.getDevice("RFoot/Bumper/Right")
lfootlbumper.enable(timestep)
lfootrbumper.enable(timestep)
rfootlbumper.enable(timestep)
rfootrbumper.enable(timestep)

# Debug log for fall detection
DEBUG_LOG_PATH = os.path.join(os.path.dirname(__file__), "debug_fall.log")
debug_log = open(DEBUG_LOG_PATH, "a", buffering=1)

# Baseline orientation for fall detection
baseline_roll = None
baseline_pitch = None
is_fallen = False

def log_fall_event(event, roll, pitch, yaw):
    timestamp = datetime.now().isoformat(timespec="seconds")
    debug_log.write(f"{timestamp} {event} roll={roll:.3f} pitch={pitch:.3f} yaw={yaw:.3f}\n")


def log_debug(message):
    timestamp = datetime.now().isoformat(timespec="seconds")
    debug_log.write(f"{timestamp} DEBUG {message}\n")

# Motion files for walking/turning (built-in Webots NAO motions)
def load_motion(path):
    if not os.path.isfile(path):
        print(f"Motion missing: {path}")
        return None
    try:
        return Motion(path)
    except Exception as e:
        print(f"Motion load error: {e}")
        return None

print(f"Using MOTION_DIR: {MOTION_DIR}")
forwards_path = os.path.join(MOTION_DIR, "Forwards50.motion")
turn_left_path = os.path.join(MOTION_DIR, "TurnLeft60.motion")
turn_right_path = os.path.join(MOTION_DIR, "TurnRight60.motion")

forwards_motion = load_motion(forwards_path)
turn_left_motion = load_motion(turn_left_path)
turn_right_motion = load_motion(turn_right_path)

if forwards_motion is None or turn_left_motion is None or turn_right_motion is None:
    print(f"Expected motions under: {MOTION_DIR}")

current_motion = None
motion_warning_printed = False

# ============================================================================
# STATE MACHINE STATES
# ============================================================================
STATE_SEARCH = "search"      # Looking for yellow boxes
STATE_APPROACH = "approach"  # Moving towards box
STATE_PICKUP = "pickup"      # Picking up the box
STATE_THROW = "throw"        # Throwing the box

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def get_active_camera():
    """Choose which camera to use based on head pitch / lost track state."""
    # Use bottom camera when looking down or when target is lost
    try:
        pitch = head_pitch.getTargetPosition()
    except Exception:
        pitch = 0.0
    if pitch > 0.6:
        return camera_bottom
    return camera_top


def get_yellow_percentage():
    """Return estimated yellow pixel percentage (0.0 to 1.0)."""
    camera = get_active_camera()
    if not camera:
        return 0.0

    try:
        image = camera.getImage()
        if not image:
            return 0.0

        width = camera.getWidth()
        height = camera.getHeight()

        # Check entire image for yellow
        yellow_count = 0
        total_pixels = width * height

        for y in range(0, height, 4):  # Skip every 4 pixels for performance
            for x in range(0, width, 4):
                r = camera.imageGetRed(image, width, x, y)
                g = camera.imageGetGreen(image, width, x, y)
                b = camera.imageGetBlue(image, width, x, y)

                # Yellow detection: high R and G, low B
                if r > 100 and g > 100 and b < 80 and (r + g) > 2.5 * b:
                    yellow_count += 1

        yellow_percentage = (yellow_count * 16) / total_pixels  # *16 because we skip pixels
        return yellow_percentage
    except Exception as e:
        print(f"Detection error: {e}")
        return 0.0


def estimate_distance_to_yellow():
    """Estimate distance (meters) from yellow area percentage.
    Uses an inverse-square-like heuristic: distance ~ scale / sqrt(area).
    """
    area = get_yellow_percentage()
    if area <= 0.0:
        return None
    distance = DISTANCE_SCALE / math.sqrt(area)
    return max(min(distance, DISTANCE_MAX), DISTANCE_MIN)


def foot_bumper_pressed():
    """Return True if any foot bumper is pressed (contact detected)."""
    return (
        lfootlbumper.getValue() > 0.0 or
        lfootrbumper.getValue() > 0.0 or
        rfootlbumper.getValue() > 0.0 or
        rfootrbumper.getValue() > 0.0
    )


def get_yellow_centroid():
    """Return normalized (x, y) centroid of yellow pixels, or None if not found."""
    camera = get_active_camera()
    if not camera:
        return None
    try:
        image = camera.getImage()
        if not image:
            return None

        width = camera.getWidth()
        height = camera.getHeight()

        sum_x = 0
        sum_y = 0
        count = 0

        for y in range(0, height, 4):
            for x in range(0, width, 4):
                r = camera.imageGetRed(image, width, x, y)
                g = camera.imageGetGreen(image, width, x, y)
                b = camera.imageGetBlue(image, width, x, y)

                if r > 100 and g > 100 and b < 80 and (r + g) > 2.5 * b:
                    sum_x += x
                    sum_y += y
                    count += 1

        if count == 0:
            return None

        cx = sum_x / count
        cy = sum_y / count

        # Normalize to [-1, 1]
        nx = (cx / (width - 1)) * 2 - 1
        ny = (cy / (height - 1)) * 2 - 1
        return nx, ny
    except Exception:
        return None


def track_yellow_with_head():
    """Adjust head yaw/pitch to keep yellow object centered."""
    centroid = get_yellow_centroid()
    global lost_track_pitch
    if centroid is None:
        # Slowly lower head to try to keep the object in view
        try:
            current_pitch = head_pitch.getTargetPosition()
        except Exception:
            current_pitch = 0.0

        lost_track_pitch = min(1.5, current_pitch + 0.08)
        head_pitch.setPosition(lost_track_pitch)
        if debug_step % 20 == 0:
            log_debug(f"TRACK_LOST lowering pitch={lost_track_pitch:.2f}")
        return

    nx, ny = centroid
    # Proportional control for head tracking
    yaw_target = -HEAD_TRACK_YAW_GAIN * nx
    pitch_target = HEAD_TRACK_PITCH_GAIN * ny + 0.15

    # Clamp to safe range
    yaw_target = max(min(yaw_target, 1.2), -1.2)
    pitch_target = max(min(pitch_target, 1.0), -0.6)

    # Reset lost-track bias when target is visible
    lost_track_pitch = 0.0

    # Smooth head movement with step limiting
    try:
        current_yaw = head_yaw.getTargetPosition()
        current_pitch = head_pitch.getTargetPosition()
    except Exception:
        current_yaw = 0.0
        current_pitch = 0.0

    target_yaw = current_yaw + (yaw_target - current_yaw) * HEAD_TRACK_SMOOTHING
    target_pitch = current_pitch + (pitch_target - current_pitch) * HEAD_TRACK_SMOOTHING

    # Limit step size per timestep for smooth motion
    yaw_step = max(min(target_yaw - current_yaw, HEAD_TRACK_MAX_STEP), -HEAD_TRACK_MAX_STEP)
    pitch_step = max(min(target_pitch - current_pitch, HEAD_TRACK_MAX_STEP), -HEAD_TRACK_MAX_STEP)

    head_yaw.setPosition(current_yaw + yaw_step)
    head_pitch.setPosition(current_pitch + pitch_step)


def detect_yellow_object():
    """Return True if a yellow object is detected."""
    yellow_percentage = get_yellow_percentage()
    if yellow_percentage >= YELLOW_DETECT_PERCENT:
        print(f"Yellow detected: {yellow_percentage*100:.2f}%")
        return True
    return False


def start_motion(motion):
    """Start a motion if not already playing it."""
    global current_motion
    if motion is None:
        return
    try:
        current_over = current_motion is None or current_motion.isOver()
    except Exception:
        current_over = True

    if current_motion is None or current_motion != motion or current_over:
        try:
            if current_motion and not current_motion.isOver():
                current_motion.stop()
        except Exception:
            pass
        try:
            motion.play()
            current_motion = motion
        except Exception:
            current_motion = None


def stop_motion():
    """Stop any current motion."""
    global current_motion
    try:
        if current_motion and not current_motion.isOver():
            current_motion.stop()
    except Exception:
        pass
    current_motion = None


def set_walk_stance():
    """Stop walking motions and keep the robot steady."""
    stop_motion()


def move_forward():
    """Move the robot forward using a built-in motion."""
    global motion_warning_printed
    if forwards_motion is None:
        if not motion_warning_printed:
            print("Forward motion not available. Check WEBOTS_HOME or MOTION_DIR.")
            motion_warning_printed = True
        return
    start_motion(forwards_motion)


def turn_left():
    """Turn the robot left using a built-in motion."""
    start_motion(turn_left_motion)


def turn_right():
    """Turn the robot right using a built-in motion."""
    start_motion(turn_right_motion)


def stop():
    """Stop the robot movement and return to steady state."""
    stop_motion()


def look_forward():
    """Look forward (neutral head position)."""
    head_yaw.setPosition(0.0)
    head_pitch.setPosition(0.0)


def look_up():
    """Look upward."""
    head_pitch.setPosition(-0.3)


def look_down():
    """Look downward."""
    head_pitch.setPosition(0.3)


def look_left():
    """Look to the left."""
    head_yaw.setPosition(0.5)


def look_right():
    """Look to the right."""
    head_yaw.setPosition(-0.5)


def move_arm_to_rest():
    """Move arm to neutral/rest position."""
    r_shoulder_pitch.setPosition(0.0)
    r_shoulder_roll.setPosition(0.0)
    r_elbow_roll.setPosition(0.0)
    r_elbow_yaw.setPosition(0.0)
    r_wrist_yaw.setPosition(0.0)


def move_arm_to_lowered():
    """Move both arms to lowered starting position."""
    # Right arm
    r_shoulder_pitch.setPosition(1.0)   # Lower down
    r_shoulder_roll.setPosition(0.3)    # Slightly out
    r_elbow_roll.setPosition(0.8)       # Bent
    r_elbow_yaw.setPosition(0.0)        # Neutral
    r_wrist_yaw.setPosition(0.0)        # Neutral
    
    # Left arm
    l_shoulder_pitch.setPosition(1.0)   # Lower down
    l_shoulder_roll.setPosition(-0.3)   # Slightly out (opposite side)
    l_elbow_roll.setPosition(-0.8)      # Bent (opposite side)
    l_elbow_yaw.setPosition(0.0)        # Neutral
    l_wrist_yaw.setPosition(0.0)        # Neutral


def bend_for_pickup():
    """Bend torso and legs to reach low objects."""
    l_hip_pitch.setPosition(-0.8)
    r_hip_pitch.setPosition(-0.8)
    l_knee_pitch.setPosition(1.2)
    r_knee_pitch.setPosition(1.2)
    l_ankle_pitch.setPosition(-0.6)
    r_ankle_pitch.setPosition(-0.6)


def set_body_tilt(tilt_forward):
    """Tilt body forward by setting hip/ankle pitch."""
    tilt_forward = max(min(tilt_forward, APPROACH_TILT_TARGET), 0.0)
    l_hip_pitch.setPosition(-0.35 - tilt_forward)
    r_hip_pitch.setPosition(-0.35 - tilt_forward)
    l_ankle_pitch.setPosition(-0.3 - tilt_forward * 0.4)
    r_ankle_pitch.setPosition(-0.3 - tilt_forward * 0.4)


def open_arms_wide():
    """Open both arms for grasping posture."""
    r_shoulder_pitch.setPosition(0.8)
    r_shoulder_roll.setPosition(0.9)
    r_elbow_roll.setPosition(0.6)
    r_elbow_yaw.setPosition(0.0)
    r_wrist_yaw.setPosition(0.0)

    l_shoulder_pitch.setPosition(0.8)
    l_shoulder_roll.setPosition(-0.9)
    l_elbow_roll.setPosition(-0.6)
    l_elbow_yaw.setPosition(0.0)
    l_wrist_yaw.setPosition(0.0)


def close_arms_in():
    """Close both arms to grasp."""
    r_shoulder_pitch.setPosition(1.1)
    r_shoulder_roll.setPosition(0.1)
    r_elbow_roll.setPosition(1.0)
    r_elbow_yaw.setPosition(0.0)
    r_wrist_yaw.setPosition(0.0)

    l_shoulder_pitch.setPosition(1.1)
    l_shoulder_roll.setPosition(-0.1)
    l_elbow_roll.setPosition(-1.0)
    l_elbow_yaw.setPosition(0.0)
    l_wrist_yaw.setPosition(0.0)


def move_arm_to_pickup():
    """Move both arms down to pickup position."""
    # Right arm
    r_shoulder_pitch.setPosition(1.1)   # Reach down
    r_shoulder_roll.setPosition(0.3)    # Slightly out
    r_elbow_roll.setPosition(0.9)       # Bend elbow (within limits)
    r_elbow_yaw.setPosition(0.0)        # Neutral
    r_wrist_yaw.setPosition(0.0)        # Neutral

    # Left arm
    l_shoulder_pitch.setPosition(1.1)   # Reach down
    l_shoulder_roll.setPosition(-0.3)   # Slightly out (opposite)
    l_elbow_roll.setPosition(-0.9)      # Bend elbow (opposite)
    l_elbow_yaw.setPosition(0.0)        # Neutral
    l_wrist_yaw.setPosition(0.0)        # Neutral


def move_arm_to_throw():
    """Move arm up to throw position."""
    r_shoulder_pitch.setPosition(-0.8)  # Raise up
    r_shoulder_roll.setPosition(-0.3)   # Out to side
    r_elbow_roll.setPosition(0.3)       # Extended
    r_elbow_yaw.setPosition(0.0)        # Neutral
    r_wrist_yaw.setPosition(0.0)        # Open hand


# ============================================================================
# MAIN CONTROL LOOP
# ============================================================================
# Initialize robot posture
move_arm_to_lowered()
set_walk_stance()

state = STATE_SEARCH
search_time = 0
action_timer = 0
look_direction = 0  # 0=forward, 1=left, 2=right, 3=up, 4=down
approach_tilt = 0.0
target_locked = False
debug_step = 0
lost_track_pitch = 0.0

while robot.step(timestep) != -1:
    action_timer += timestep
    debug_step += 1
    if target_locked:
        track_yellow_with_head()
    # Fall detection
    roll, pitch, yaw = inertial_unit.getRollPitchYaw()
    if baseline_roll is None:
        baseline_roll = roll
        baseline_pitch = pitch
        log_fall_event("BASELINE", roll, pitch, yaw)

    roll_delta = abs(roll - baseline_roll)
    pitch_delta = abs(pitch - baseline_pitch)

    if not is_fallen and (roll_delta > FALL_ANGLE_THRESHOLD or pitch_delta > FALL_ANGLE_THRESHOLD):
        is_fallen = True
        log_fall_event("FALL", roll, pitch, yaw)
    elif is_fallen and (roll_delta < FALL_ANGLE_THRESHOLD * 0.6 and pitch_delta < FALL_ANGLE_THRESHOLD * 0.6):
        is_fallen = False
        log_fall_event("RECOVER", roll, pitch, yaw)
    
    # STATE: SEARCH
    if state == STATE_SEARCH:
        if detect_yellow_object():
            state = STATE_APPROACH
            action_timer = 0
            track_yellow_with_head()
            target_locked = True
            print("Yellow object found! Approaching...")
            log_debug("TARGET_LOCKED")
        else:
            if target_locked:
                track_yellow_with_head()
            else:
                # Smooth sweeping search pattern with multiple positions
                cycle_time = action_timer % 12000  # 12 second cycle
                
                if cycle_time < 2000:
                    # Pan left to right at neutral height
                    head_yaw.setPosition(-0.5 + (cycle_time / 2000.0))
                    head_pitch.setPosition(0.0)
                elif cycle_time < 4000:
                    # Pan right to left at neutral height
                    head_yaw.setPosition(0.5 - ((cycle_time - 2000) / 2000.0))
                    head_pitch.setPosition(0.0)
                elif cycle_time < 6000:
                    # Pan left to right while looking down
                    head_yaw.setPosition(-0.5 + ((cycle_time - 4000) / 2000.0))
                    head_pitch.setPosition(0.4)
                elif cycle_time < 8000:
                    # Pan right to left while looking down
                    head_yaw.setPosition(0.5 - ((cycle_time - 6000) / 2000.0))
                    head_pitch.setPosition(0.4)
                elif cycle_time < 10000:
                    # Pan left to right while looking up
                    head_yaw.setPosition(-0.5 + ((cycle_time - 8000) / 2000.0))
                    head_pitch.setPosition(-0.3)
                else:
                    # Pan right to left while looking up
                    head_yaw.setPosition(0.5 - ((cycle_time - 10000) / 2000.0))
                    head_pitch.setPosition(-0.3)
    
    # STATE: APPROACH
    elif state == STATE_APPROACH:
        # Move towards the object until it appears close enough
        yellow_percentage = get_yellow_percentage()
        distance_est = estimate_distance_to_yellow()
        if debug_step % 20 == 0:
            cam_name = "bottom" if get_active_camera() == camera_bottom else "top"
            log_debug(f"APPROACH cam={cam_name} yellow={yellow_percentage:.4f} dist={distance_est}")

        if foot_bumper_pressed():
            # Contact detected
            state = STATE_PICKUP
            action_timer = 0
            stop()
            log_debug("FOOT_CONTACT -> PICKUP")
        else:
            track_yellow_with_head()
            # Always walk forward while approaching
            move_forward()
            # If we lose it, keep tracking and walking forward slowly
            if yellow_percentage < YELLOW_DETECT_PERCENT and action_timer > 4000:
                log_debug("LOST_TARGET -> CONTINUE")
    
    # STATE: PICKUP
    elif state == STATE_PICKUP:
        # Stage 1: lower knees slowly and open arms (stable)
        if action_timer < PICKUP_STAGE_TIME:
            track_yellow_with_head()
            set_body_tilt(0.15)
            l_hip_pitch.setPosition(-0.6)
            r_hip_pitch.setPosition(-0.6)
            l_knee_pitch.setPosition(1.05)
            r_knee_pitch.setPosition(1.05)
            l_ankle_pitch.setPosition(-0.4)
            r_ankle_pitch.setPosition(-0.4)
            open_arms_wide()
        # Stage 2: lower knees further, keep torso stable, reach down
        elif action_timer < PICKUP_STAGE_TIME * 2:
            track_yellow_with_head()
            set_body_tilt(0.2)
            l_hip_pitch.setPosition(-0.8)
            r_hip_pitch.setPosition(-0.8)
            l_knee_pitch.setPosition(1.35)
            r_knee_pitch.setPosition(1.35)
            l_ankle_pitch.setPosition(-0.55)
            r_ankle_pitch.setPosition(-0.55)
            move_arm_to_pickup()
        # Stage 3: close arms slowly to grasp
        elif action_timer < PICKUP_STAGE_TIME * 4:
            track_yellow_with_head()
            set_body_tilt(0.2)
            close_arms_in()
        else:
            state = STATE_THROW
            action_timer = 0
    
    # STATE: THROW (repurposed as TURN_AROUND)
    elif state == STATE_THROW:
        # Slowly turn around after pickup
        turn_right()
        if action_timer > 4000:
            stop()
            move_arm_to_rest()
            state = STATE_SEARCH
            action_timer = 0


