# 🤖 NAO Robot Learns to Find & Grab

Welcome to the ultimate robot learning simulator! This project teaches a NAO humanoid robot to find a yellow rubber duck using **reinforcement learning** and **full motor control**. No pre-programmed moves, no cheating—the robot learns by trying things and getting rewards when it does something good.

This is called **Q-Learning**, and it's the OG reinforcement learning algorithm that powers everything from game-playing AIs to real robot control.

## 🎯 What's Different Now?

Your robot isn't playing motion files anymore. **It controls every motor directly**: head, arms, legs, everything. It's learning actual robot behavior from scratch.

- **11 Different Actions**: Look left/right, reach forward, grab, walk in all directions
- **Full Motor Control**: Not just playing pre-recorded kicks—learning real arm kinematics
- **Persistent Training**: Episode counter saves, so training resumes where it left off
- **Manual Scoring**: YOU can reward or penalize the robot in real-time to guide learning
- **Detailed Logging**: See exactly what the robot tries at each step

## 🧠 How Does It Learn?

### The Loop (Repeat Until Success)

1. **See** – NAO looks through its cameras
2. **Think** – "Where's the yellow duck? What should I do?"
3. **Act** – Execute one of 11 motor actions
4. **Feel** – Get automatic reward or YOU manually score it
5. **Learn** – Update the Q-table (memory of good/bad moves)
6. **Reset** – Put everything back, continue

### The Scoring System

| What Happened | Automatic Reward |
|---|---|
| Every step (time cost) | -0.02 |
| Getting closer to yellow | +0.3 × (visibility increase) |
| Duck lifts off ground | +4.0 |
| Grabbing & moving duck | +6.0 |
| **YOU press +** | **+5.0 (manual bonus)** |
| **YOU press -** | **-5.0 (manual penalty)** |

The robot learns: *"I want that +5-6 reward, so I need to find the yellow thing, reach toward it, and grab!"*

### The 11 Actions

- **head_left** – Look left (YAW motor)
- **head_right** – Look right (YAW motor)
- **head_center** – Look straight (reset YAW)
- **arm_forward** – Reach arm forward (SHOULDER + ELBOW)
- **arm_down** – Lower arm to rest position
- **arm_up** – Raise arm
- **walk_forward** – Step forward (HIP + KNEE motors)
- **walk_left** – Sidestep left (HIP ROLL)
- **walk_right** – Sidestep right (HIP ROLL)
- **walk_backward** – Step backward (HIP motors)
- **grab** – Close fingers (WRIST YAW)

### The Brain: Q-Table

The **Q-Table** is like a memo book:

```
State: (yellow_visibility, head_angle)
└─ turn_left: -0.5 (bad idea)
└─ turn_right: +0.8 (pretty good!)
└─ forward: +0.3 (okay)
└─ kick: -2.0 (terrible, nothing to kick)
```

Over time, the values get better. The robot learns which action is best in each situation.

## 📸 Vision-Based Learning

No cheating! The robot sees the world through:
- **CameraTop** – looking forward and down
- **CameraBottom** – looking more downward

It scans for **yellow pixels** (the duck). The more yellow it sees, the closer it is. Simple!

```python
# Detecting yellow
if red > 150 and green > 150 and blue < 100:
    # That's yellow!
    yellow_count += 1
```

## 🏃 How to Run It

1. Open **`worlds/bobby.wbt`** in Webots
2. Press **Play** (▶)
3. Watch the console output—it shows exactly what the robot does each step
4. Optionally: Guide learning by manually scoring with + and -

**The robot remembers everything:**
- `q_table.json` – Learned behavior (updated every episode)
- `episode.json` – Current episode number (so training resumes where it left off)

Even if you pause, reset, or restart Webots, the robot picks up right where it left off!

## 🎓 Manual Scoring (YOU are the Coach!)

When the robot does something smart:
```
Step 5: yellow=0.45% -> action=arm_forward
  [+ to reward +5] [- to penalize -5]
```

You can press:
- **+** to give +5 bonus (robot did something good!)
- **-** to give -5 penalty (robot did something bad!)

This guides training faster without waiting for slow automatic rewards.

## 📊 Progress Tracking

Every episode prints:
```
=== Episode 15/100 ===
  Step 1: yellow=0.05% -> action=head_left
    Reward: -0.02
  Step 2: yellow=0.12% -> action=arm_forward
    Reward: +0.3
  ...
Episode 15 total_reward=1.45
```

Watch the `total_reward` grow as the robot learns! Higher = smarter.

## 🎮 The Actions (Updated for Motor Control)

The robot now controls **real motors** instead of playing motion files:

| Action | Motors | Effect |
|--------|--------|--------|
| **head_left** | HeadYaw | Turn head to look left |
| **head_right** | HeadYaw | Turn head to look right |
| **head_center** | HeadYaw | Center head position |
| **arm_forward** | RShoulderPitch, RElbowRoll | Reach arm forward |
| **arm_down** | RShoulderPitch, RElbowRoll | Rest arm down |
| **arm_up** | RShoulderPitch, RElbowRoll | Raise arm up |
| **walk_forward** | LHipPitch, RHipPitch, LKneePitch, RKneePitch | Step forward |
| **walk_left** | LHipRoll, RHipRoll | Sidestep left |
| **walk_right** | LHipRoll, RHipRoll | Sidestep right |
| **walk_backward** | LHipPitch, RHipPitch (reversed) | Step backward |
| **grab** | RWristYaw | Close/open fingers |

This is **way more realistic** than pre-recorded motions!

## 🔧 Tuning It Up

### Wanna change the learning speed?

Edit [controllers/NAO_RL_Kick/NAO_RL_Kick.py](controllers/NAO_RL_Kick/NAO_RL_Kick.py):

```python
MAX_EPISODES = 100         # How many training runs
MAX_STEPS = 100           # Steps per episode
ALPHA = 0.1               # Learning rate (higher = learns faster)
GAMMA = 0.95              # Future reward weight (plans ahead more)
EPSILON = 0.15            # Exploration rate (tries random moves)
```

### Wanna change motor movements?

Find `execute_action()` and tweak the setPosition() values:
- Lower number = more curled (negative angles)
- Higher number = more extended (positive angles)

For example, `arm_forward`:
```python
r_shoulder_pitch.setPosition(0.8)   # How far forward?
r_elbow_roll.setPosition(-2.0)      # How extended?
step_for(400)                        # How long to hold?
```

### Wanna change rewards?

Find `reward_for()` function:
```python
reward += 0.3 * (new_yellow - prev_yellow)  # Approach bonus
if duck_height > 0.05:
    reward += 4.0                           # Pickup bonus
if action == "grab" and duck_moved > 0.1:
    reward += 6.0                           # Grab success
```

Bigger numbers = robot cares more about that goal.

## 📁 Project Structure

```
controllers/
├── NAO_RL_Kick/
│   ├── NAO_RL_Kick.py      # Main RL trainer (supervisor)
│   ├── q_table.json        # Learned brain (auto-saved every episode)
│   └── episode.json        # Current episode number (persists across resets)
└── [other controllers]

worlds/
└── bobby.wbt              # The simulation world
    ├── NAO robot (supervisor with full motor control)
    └── RubberDuck (yellow target at position 0.31, 0, 0.02)
```

## 🎓 The Algorithm: Q-Learning Explained

For nerds, the update is:

$$Q(s, a) \leftarrow Q(s, a) + \alpha \left[ r + \gamma \max_a Q(s', a) - Q(s, a) \right]$$

**In English:**
- `Q(s, a)` = "How good is action `a` in state `s`?"
- `r` = reward we just got
- `s'` = new state after action
- `α` = learning rate (how much we update)
- `γ` = discount factor (how much we care about future)

The robot adjusts its memory based on: *reward I got + best future action - what I thought*

If reality is better than expected, it loves that action more. If worse, it avoids it.

## 🚀 What's Next?

- **Faster Training** – Increase MAX_EPISODES, tune ALPHA/GAMMA
- **Add Constraints** – Robot can only use 3 actions (look, reach, grab) to make learning simpler
- **Real Obstacles** – Add walls and teach navigation
- **Multiple Targets** – Different objects, different behaviors
- **Simulation → Real Robot** – Deploy to actual NAO hardware
- **Human Feedback** – Use the +/- scoring to shape behavior without waiting for auto-rewards

## 🐛 Troubleshooting

**"Yellow detection not working"**
- Check the duck is visible in camera
- Tweak RGB thresholds in `get_yellow_percentage()`

**"Robot not improving"**
- Maybe rewards are too weak? Try bigger numbers
- Reduce EPSILON to focus on best actions
- Increase MAX_EPISODES

**"Controller crashes"**
- Make sure DEF NAO and DEF DUCK are in the world file
- Ensure supervisor=TRUE on NAO

## 🎬 Fun Ideas

- **Race Mode** – Two NAOs, first to kick wins
- **Obstacle Course** – Add walls, see if it learns to navigate
- **Color Learning** – Teach it different colors = different actions
- **Multi-kick** – Reward chaining kicks together

## 📚 References

- **Q-Learning**: Watkins & Dayan (1992) - "Q-learning"
- **Webots Docs**: https://cyberbotics.com/
- **RL Basics**: Sutton & Barto - "Reinforcement Learning: An Introduction"

---

**Made with 🤖 and 💛 (the duck is yellow, after all)**

Happy training! 🚀
