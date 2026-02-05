from controller import Robot, Motion
import os

robot = Robot()
timestep = int(robot.getBasicTimeStep())

receiver = robot.getDevice("receiver")
if receiver is None:
    receiver = robot.getDevice("RECEIVER")

if receiver is None:
    print("Receiver device not found. Ensure NAO channel is set and receiver exists.")
else:
    receiver.enable(timestep)

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


motions = {
    "forward": load_motion("Forwards50.motion"),
    "turn_left": load_motion("TurnLeft60.motion"),
    "turn_right": load_motion("TurnRight60.motion"),
    "kick": load_motion("KickRight.motion"),
}

current_motion = None
current_action = None


def start_motion(action):
    global current_motion, current_action
    motion = motions.get(action)
    if motion is None:
        return
    motion.play()
    current_motion = motion
    current_action = action


while robot.step(timestep) != -1:
    if receiver is not None:
        while receiver.getQueueLength() > 0:
            message = receiver.getData().decode("utf-8")
            receiver.nextPacket()
            start_motion(message)

    if current_motion is not None and current_motion.isOver():
        current_motion.stop()
        current_motion = None
        current_action = None
