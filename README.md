<<<<<<< HEAD
<<<<<<< HEAD
# BTP_Turtle_Door
=======
# Door State Estimation using Bayes Filter
=======
# door_state_estimation
>>>>>>> Initial commit: door_state_estimation package with README

This package contains a ROS node that estimates the state of a door using a Bayesian filter, visualized in turtlesim. The robot (red turtle) moves in a straight line to three doors (blue turtles), updating its belief about each door's state based on user actions and sensor observations.

## How the Code Works

- **Robot Turtle**: The default turtle (`turtle1`) is used as the robot, starting at the left-middle of the turtlesim grid.
- **Doors**: Three doors are spawned in a straight horizontal line in front of the robot. Each door is represented by a blue turtle.
- **Movement**: The robot moves straight to each door. If its belief that the door is open exceeds 0.5, it passes through; otherwise, it waits for further user input.
- **User Interaction**: For each door, the user is prompted to enter an action (`push` or `do nothing`) and a sensor observation (`open` or `closed`).
- **Bayesian Filter**: The robot updates its belief using a Bayes filter:
  - **Prediction Step**: Updates belief based on the action and motion model.
  - **Correction Step**: Updates belief based on the sensor observation and sensor model.
  - **Normalization**: Ensures beliefs sum to 1.0.
- **Logging**: All calculations (probabilities, eta, normalized beliefs) are logged to `src/door_bayes_log.txt` for each door.

## Calculation Details

### State Space
- **States**: `open`, `closed`
- **Actions**: `push`, `do nothing`
- **Observations**: `open`, `closed`

### Probabilistic Models

**Sensor Model** - P(observation | true_state):
```
P(sensor="open" | door=open) = 0.6
P(sensor="closed" | door=open) = 0.4
P(sensor="open" | door=closed) = 0.2
P(sensor="closed" | door=closed) = 0.8
```

**Motion Model** - P(next_state | action, current_state):
```
P(open | push, closed) = 0.8
P(closed | push, closed) = 0.2
P(open | push, open) = 1.0
P(closed | push, open) = 0.0

P(open | do_nothing, open) = 1.0
P(closed | do_nothing, open) = 0.0
P(open | do_nothing, closed) = 0.0
P(closed | do_nothing, closed) = 1.0
```

### Bayesian Filter Steps

1. **Prediction Step**
   - Uses the motion model to predict the new belief after an action.
   - Example:
     ```python
     bel_open = 0.8 * bel_closed_prev + 1.0 * bel_open_prev
     bel_closed = 0.2 * bel_closed_prev + 0.0 * bel_open_prev
     eta = 1.0 / (bel_open + bel_closed)
     bel_open /= (bel_open + bel_closed)
     bel_closed /= (bel_open + bel_closed)
     ```
2. **Correction Step**
   - Uses the sensor model to update the belief based on the observation.
   - Example:
     ```python
     bel_open *= 0.6  # if observation is 'open'
     bel_closed *= 0.2
     eta = 1.0 / (bel_open + bel_closed)
     bel_open /= (bel_open + bel_closed)
     bel_closed /= (bel_open + bel_closed)
     ```

All steps and intermediate values are logged for each door.

## How to Run the Code

```bash
# 1. Build the workspace
cd ~/catkin_ws
catkin_make

# 2. Source the workspace
source devel/setup.bash

# 3. Make sure the script is executable
chmod +x src/door_state_estimation/scripts/turtle_door_bayes.py

# 4. Start ROS core (Terminal 1)
roscore

# 5. Start turtlesim (Terminal 2)
rosrun turtlesim turtlesim_node

# 6. Run the turtle door Bayes filter demo (Terminal 3)
rosrun door_state_estimation turtle_door_bayes.py
```

## How to clone and run this code in a new workspace

```bash
# Clone the repository
cd ~
git clone https://github.com/Sachin22424/BTP_Robot_Door.git

# Create a new catkin workspace and add the package
mkdir -p ~/catkin_ws/src
cp -r ~/BTP_Robot_Door/door_state_estimation ~/catkin_ws/src/

# Build and run as above
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x src/door_state_estimation/scripts/turtle_door_bayes.py
roscore
rosrun turtlesim turtlesim_node
rosrun door_state_estimation turtle_door_bayes.py
```

## Example Log File

After running, check `src/door_bayes_log.txt` for detailed belief calculations for each door, e.g.:
```
Door 1 (door1) at (4,5.5):
Prediction step:
  open = 0.8 * 0.500 + 1.0 * 0.500 = 0.900
  closed = 0.2 * 0.500 + 0.0 * 0.500 = 0.100
  eta = 1/(open+closed) = 1.000
  normalized: open = 0.900, closed = 0.100
Correction step:
  open = 0.6 * 0.900 = 0.540
  closed = 0.2 * 0.100 = 0.020
  eta = 1/(open+closed) = 1.786
  normalized: open = 0.964, closed = 0.036
  Passed through door1.
```

## Troubleshooting
- If you change any Python script, you do **not** need to run `catkin_make` unless you change CMakeLists.txt or want to update install targets.
- Always source the workspace after building: `source devel/setup.bash`
- If you get permission errors, run: `chmod +x src/door_state_estimation/scripts/turtle_door_bayes.py`

## Contributing
Feel free to extend this implementation with:
- More sophisticated sensor models
- Additional door states
- Visualization tools
<<<<<<< HEAD
- Parameter tuning interfaces

## 📄 License

MIT License - Feel free to use and modify for educational purposes.
>>>>>>> Initial commit: Bayes Filter for Door State Estimation
=======
- Parameter tuning interfaces
>>>>>>> Updated readme
