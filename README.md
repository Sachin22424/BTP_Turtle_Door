# Multi-Robot Door State Estimation using Bayes Filter

This package contains a ROS node that demonstrates multi-robot door state estimation using Bayesian filters, visualized in turtlesim. Two robots (red and green turtles) independently explore doors scattered across the environment, sharing beliefs and updating their estimates using Bayes filters.

## Latest Improvements (August 2025)

### 🚀 **New Features in `bayes_filter_node_new.py`**

- **Independent Multi-Robot Exploration**: Two robots work independently, choosing doors based on proximity
- **Distributed Door Layout**: Doors positioned at different corners instead of a straight line
- **Enhanced Visualization**: 
  - Robot1 (Red) starts at bottom-left corner (1,1)
  - Robot2 (Green) starts at top-right corner (10,10)
  - Doors color-coded: Green=Open, Magenta=Closed
  - Smooth movement with better path visualization
- **Intelligent Belief Sharing**: Robots share beliefs after each door interaction
- **Threshold-Based Stopping**: Robots stop when belief confidence crosses individual thresholds

## How the Code Works

### Multi-Robot System
- **Robot1 (Red)**: Starts at (1.0, 1.0) with threshold=0.6, comm_weight=0.3
- **Robot2 (Green)**: Starts at (10.0, 10.0) with threshold=0.5, comm_weight=0.4
- **Door Distribution**: 5 doors positioned at: (3,3), (8,2), (2,8), (9,7), (5.5,5.5)

### Movement Strategy
- Each robot finds its closest unvisited door
- Smooth movement with controlled approach distance (0.5 units from door)
- Independent exploration - no sequential door visiting

### Bayesian Filter Process
1. **Action Decision**: Robot decides to "push" or "do nothing" based on current belief
2. **Door Interaction**: Simulate pushing with 70% success probability for closed doors
3. **Belief Update**: Apply Bayes filter with prediction and correction steps
4. **Belief Sharing**: Share updated beliefs with other robots using weighted averaging
5. **Threshold Check**: Stop when all doors exceed belief thresholds
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

# 4. Direct
roslaunch door_state_estimation multi_robot_door_bayes.launch

Other way

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
