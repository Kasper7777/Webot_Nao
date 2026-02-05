# 🤖 NAO Robot Learns to Kick Yellow Things

Welcome to the ultimate robot soccer training simulator! This project teaches a NAO robot to find and kick a yellow rubber duck using **reinforcement learning** and **camera vision**. No pre-programmed moves, no cheating with supervisor coordinates—just pure AI learning.

## 🎯 What's Happening Here?

Your robot is learning like a toddler learning to walk. It doesn't know what "kick" means yet. But every time it does something good, it gets a tiny reward. Every time it does something dumb, it loses points. Over hundreds of tries, it figures out: *"Oh! If I turn toward the yellow thing and then punch, it moves! That's good!"*

This is called **Q-Learning**, and it's the OG reinforcement learning algorithm that powers everything from game-playing AIs to real robot control.

## 🧠 How Does It Learn?

### The Loop (Repeat 200 times)

1. **See** – NAO looks at the world through its cameras
2. **Think** – "Where's the yellow duck? What should I do?"
3. **Act** – Turn left, turn right, walk forward, or KICK
4. **Feel** – Get a reward or penalty for the action
5. **Learn** – Update the Q-table (memory of good/bad moves)
6. **Reset** – Put everything back, start over

### The Scoring System

| What Happened | Reward |
|---|---|
| Every step (just existing) | -0.01 |
| Getting closer to yellow | +0.2 |
| Duck lifts off ground | +3.0 |
| Kicking duck far away | +5.0 |

The robot learns: *"I want that +5 kick reward, so I need to find the yellow thing first (+0.2 per step closer), then kick it!"*

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
3. Watch NAO stumble around and slowly get smarter
4. Check the console for progress: `Episode 10 reward=4.23`

The Q-table saves after every 10 episodes, so it improves even if you stop and restart!

## 📊 Progress Tracking

Every 10 episodes, you see:
```
Episode 10 reward=1.23
Episode 20 reward=2.45
Episode 30 reward=3.12
...
```

**Higher = better!** If the robot hits +5 or higher consistently, it's learned to kick!

## 🎮 The Actions

- **turn_left** – Spin counterclockwise (via TurnLeft60.motion)
- **turn_right** – Spin clockwise (via TurnRight60.motion)
- **forward** – Walk forward (via Forwards50.motion)
- **kick** – Attempt a mighty kick (via KickRight.motion)

Each action takes ~700-1000ms. The robot learns which combo gets results.

## 🔧 Tuning It Up

### Wanna change the learning speed?

Edit [controllers/NAO_RL_Kick/NAO_RL_Kick.py](controllers/NAO_RL_Kick/NAO_RL_Kick.py):

```python
ALPHA = 0.2      # Learning rate (higher = learns faster, but noisier)
GAMMA = 0.95     # Future reward weight (higher = thinks ahead more)
EPSILON = 0.1    # Exploration (higher = tries random moves more)
MAX_EPISODES = 200  # How many training runs
```

### Wanna change rewards?

Find `reward_for()` function and tweak:
- `duck_height > 0.02` – sensitivity for pickup
- `+3.0` – pickup reward
- `+5.0` – kick reward

Make them bigger = robot cares more. Make them smaller = robot less interested.

## 📁 Project Structure

```
controllers/
├── NAO_RL_Kick/
│   ├── NAO_RL_Kick.py      # Main RL trainer (supervisor)
│   └── q_table.json        # Learned brain (auto-saved)
└── NAO_RL_Kick_Agent/
    └── NAO_RL_Kick_Agent.py  # Legacy (not used in vision mode)

worlds/
└── bobby.wbt              # The simulation world
    ├── NAO robot (learning controller)
    └── RubberDuck (yellow target)
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

- **Policy Replay** – After training, run it without random exploration (EPSILON=0) to see the learned strategy
- **Curriculum Learning** – Start with duck close, gradually move it farther
- **Multi-target** – Add green, blue objects and teach different behaviors
- **Real Robot** – Deploy to actual NAO hardware!

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
