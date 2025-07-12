<<<<<<< HEAD
# BTP_Turtle_Door
=======
# Door State Estimation using Bayes Filter

This ROS package implements a Bayes filter for estimating the state of a door (open/closed) based on actions and sensor observations. The implementation follows the probabilistic robotics principles from Section 2.4.2.

## 📁 Package Structure

```
door_state_estimation/
├── scripts/
│   └── bayes_filter_node.py      # Main Bayes filter implementation
├── launch/
│   └── bayes_filter_demo.launch  # Launch file for the demo
├── CMakeLists.txt                # Build configuration
├── package.xml                   # Package dependencies
└── README.md                     # This file
```

## 🧠 Algorithm Overview

The Bayes filter estimates the probability distribution over door states using:

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

## 🔧 Prerequisites

- ROS Noetic (Ubuntu 20.04)
- Python 3
- `std_msgs` package

## 🚀 Building the Package

1. **Clone or create the workspace** (if not already done):
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ```

2. **Navigate to your workspace**:
   ```bash
   cd ~/Robo_Eg/catkin_ws
   ```

3. **Make the Python script executable**:
   ```bash
   chmod +x src/door_state_estimation/scripts/bayes_filter_node.py
   ```

4. **Build the package**:
   ```bash
   catkin_make
   ```

5. **Source the workspace**:
   ```bash
   source devel/setup.bash
   ```

## 🎯 Running the Demo

### Step 1: Start ROS Core
Open a terminal and run:
```bash
roscore
```

### Step 2: Launch the Bayes Filter Node
Open a new terminal:
```bash
cd ~/Robo_Eg/catkin_ws
source devel/setup.bash
roslaunch door_state_estimation bayes_filter_demo.launch
```

### Step 3: Publish Actions
Open a new terminal:
```bash
cd ~/Robo_Eg/catkin_ws
source devel/setup.bash

# Publish an action (choose one)
rostopic pub /door_action std_msgs/String "push"
# OR
rostopic pub /door_action std_msgs/String "do nothing"
```

### Step 4: Publish Sensor Observations
Open a new terminal:
```bash
cd ~/Robo_Eg/catkin_ws
source devel/setup.bash

# Publish a sensor observation (choose one)
rostopic pub /door_sensor std_msgs/String "open"
# OR
rostopic pub /door_sensor std_msgs/String "closed"
```

### Step 5: Monitor Belief Updates
Open a new terminal:
```bash
cd ~/Robo_Eg/catkin_ws
source devel/setup.bash
rostopic echo /door_belief
```

## 📊 Example Usage Sequence

1. **Initial state**: `belief(open)=0.5, belief(closed)=0.5`

2. **Publish action**: `"do nothing"`
   
3. **Publish sensor**: `"open"`
   - **Result**: `belief(open)=0.750, belief(closed)=0.250`

4. **Publish action**: `"push"`
   
5. **Publish sensor**: `"open"`
   - **Result**: `belief(open)=0.983, belief(closed)=0.017`

## 🔗 ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/door_action` | `std_msgs/String` | Input: Robot actions ("push", "do nothing") |
| `/door_sensor` | `std_msgs/String` | Input: Sensor observations ("open", "closed") |
| `/door_belief` | `std_msgs/String` | Output: Current belief state |

## 🛠 Troubleshooting

### Common Issues

1. **Launch file not found**:
   ```bash
   # Make sure you've built and sourced the workspace
   cd ~/Robo_Eg/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

2. **Permission denied**:
   ```bash
   # Make the script executable
   chmod +x src/door_state_estimation/scripts/bayes_filter_node.py
   ```

3. **KeyError: 'close'**:
   - Use `"closed"` instead of `"close"` for sensor observations
   - The system expects exact string matches

4. **No action yet warning**:
   - Always publish an action before publishing sensor data
   - The filter needs an action to perform the prediction step

### Resetting the Filter

To reset the belief state to initial values (0.5, 0.5):
1. Stop the node with `Ctrl+C`
2. Restart: `roslaunch door_state_estimation bayes_filter_demo.launch`

## 📝 Code Structure

The main implementation is in `scripts/bayes_filter_node.py`:

- **`DoorBayesFilter` class**: Main filter implementation
- **`action_callback()`**: Stores the latest action
- **`sensor_callback()`**: Performs Bayes update when sensor data arrives
- **Prediction step**: Uses motion model to predict belief after action
- **Correction step**: Uses sensor model to update belief based on observation
- **Normalization**: Ensures beliefs sum to 1.0

## 🔬 Mathematical Implementation

The Bayes filter follows this algorithm:

```python
# Prediction step
bel_pred = Σ P(x_t | u_t, x_{t-1}) * bel(x_{t-1})

# Correction step  
bel(x_t) = η * P(z_t | x_t) * bel_pred(x_t)

# where η is the normalization factor
```

## 📚 References

- Probabilistic Robotics by Thrun, Burgard, and Fox
- Section 2.4.2: Example of Bayes Filter Application

## 🤝 Contributing

Feel free to extend this implementation with:
- More sophisticated sensor models
- Additional door states
- Visualization tools
- Parameter tuning interfaces

## 📄 License

MIT License - Feel free to use and modify for educational purposes.
>>>>>>> Initial commit: Bayes Filter for Door State Estimation
