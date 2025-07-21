#!/usr/bin/env python3
import rospy
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import os
import sys
import time

class DoorBayesFilter:
    def __init__(self, door_name):
        self.door_name = door_name
        self.belief = {'open': 0.5, 'closed': 0.5}
        self.last_action = None
        self.log_steps = []

    def predict(self, action):
        prev_open = self.belief['open']
        prev_closed = self.belief['closed']
        if action == "push":
            open_calc = f"0.8 * {prev_closed:.3f} + 1.0 * {prev_open:.3f}"
            closed_calc = f"0.2 * {prev_closed:.3f} + 0.0 * {prev_open:.3f}"
            self.belief['open'] = 0.8 * prev_closed + 1.0 * prev_open
            self.belief['closed'] = 0.2 * prev_closed + 0.0 * prev_open
        elif action == "do nothing":
            open_calc = f"1.0 * {prev_open:.3f} + 0.0 * {prev_closed:.3f}"
            closed_calc = f"1.0 * {prev_closed:.3f} + 0.0 * {prev_open:.3f}"
            self.belief['open'] = 1.0 * prev_open + 0.0 * prev_closed
            self.belief['closed'] = 1.0 * prev_closed + 0.0 * prev_open
        total = self.belief['open'] + self.belief['closed']
        eta = 1.0 / total if total != 0 else 0
        self.belief['open'] /= total
        self.belief['closed'] /= total
        self.log_steps.append(f"Prediction step:\n  open = {open_calc} = {self.belief['open']*total:.3f}\n  closed = {closed_calc} = {self.belief['closed']*total:.3f}\n  eta = 1/(open+closed) = {eta:.3f}\n  normalized: open = {self.belief['open']:.3f}, closed = {self.belief['closed']:.3f}")

    def correct(self, observation):
        prev_open = self.belief['open']
        prev_closed = self.belief['closed']
        if observation == "open":
            open_calc = f"0.6 * {prev_open:.3f}"
            closed_calc = f"0.2 * {prev_closed:.3f}"
            self.belief['open'] *= 0.6
            self.belief['closed'] *= 0.2
        elif observation == "closed":
            open_calc = f"0.4 * {prev_open:.3f}"
            closed_calc = f"0.8 * {prev_closed:.3f}"
            self.belief['open'] *= 0.4
            self.belief['closed'] *= 0.8
        total = self.belief['open'] + self.belief['closed']
        eta = 1.0 / total if total != 0 else 0
        self.belief['open'] /= total
        self.belief['closed'] /= total
        self.log_steps.append(f"Correction step:\n  open = {open_calc} = {self.belief['open']*total:.3f}\n  closed = {closed_calc} = {self.belief['closed']*total:.3f}\n  eta = 1/(open+closed) = {eta:.3f}\n  normalized: open = {self.belief['open']:.3f}, closed = {self.belief['closed']:.3f}")

    def print_belief(self):
        print(f"Door {self.door_name}: belief(open)={self.belief['open']:.3f}, belief(closed)={self.belief['closed']:.3f}")

def set_turtle_pen(turtle_name, r, g, b, width, off):
    rospy.wait_for_service(f'/{turtle_name}/set_pen')
    set_pen = rospy.ServiceProxy(f'/{turtle_name}/set_pen', SetPen)
    set_pen(r, g, b, width, off)

def move_robot_to(x, y, pub):
    """
    Move the robot straight to the target (x, y) along a straight line.
    """
    twist = Twist()
    # Move straight in x direction (doors are aligned horizontally)
    twist.linear.x = 2.0
    twist.angular.z = 0.0
    pub.publish(twist)
    # Calculate time to reach each door based on distance
    # Assume starting at (2, 5.5), doors at (4, 5.5), (6, 5.5), (8, 5.5)
    if x == 4:
        time.sleep(1.0)
    elif x == 6:
        time.sleep(2.0)
    elif x == 8:
        time.sleep(3.0)
    twist.linear.x = 0.0
    pub.publish(twist)
    time.sleep(0.5)

def pass_through_door(pub):
    """
    Move the robot forward to simulate passing through the door.
    """
    twist = Twist()
    twist.linear.x = 2.0
    pub.publish(twist)
    time.sleep(1.5)
    twist.linear.x = 0.0
    pub.publish(twist)
    time.sleep(0.5)

def main():
    rospy.init_node('turtle_door_bayes_demo')
    rospy.wait_for_service('spawn')
    spawn = rospy.ServiceProxy('spawn', Spawn)

    # Use the initial turtle (turtle1) as the robot, place it in the middle left
    robot_name = 'turtle1'
    robot_x, robot_y = 2, 5.5
    # Teleport turtle1 to starting position
    rospy.wait_for_service(f'/{robot_name}/teleport_absolute')
    teleport = rospy.ServiceProxy(f'/{robot_name}/teleport_absolute', TeleportAbsolute)
    teleport(robot_x, robot_y, 0)
    set_turtle_pen(robot_name, 255, 0, 0, 8, 0)  # Red, thick

    # Place 3 doors in a straight line horizontally in front of robot
    doors = {
        'door1': (4, 5.5),
        'door2': (6, 5.5),
        'door3': (8, 5.5)
    }
    for name, (x, y) in doors.items():
        spawn(x, y, 0, name)
        set_turtle_pen(name, 0, 0, 255, 3, 0)  # Blue, thinner

    # Move robot to each door and update belief
    pub = rospy.Publisher(f'/{robot_name}/cmd_vel', Twist, queue_size=10)
    filters = {name: DoorBayesFilter(name) for name in doors}
    log_path = os.path.join(os.path.dirname(__file__), '../door_bayes_log.txt')
    with open(log_path, 'w') as log_file:
        for i, (name, (x, y)) in enumerate(doors.items(), 1):
            print(f"\nMoving to {name} at ({x},{y})...")
            move_robot_to(x, y, pub)
            log_file.write(f"Door {i} ({name}) at ({x},{y}):\n")
            while True:
                action = input(f"Enter action for {name} (push/do nothing): ").strip()
                filters[name].predict(action)
                obs = input(f"Enter sensor observation for {name} (open/closed): ").strip()
                filters[name].correct(obs)
                filters[name].print_belief()
                for step in filters[name].log_steps:
                    log_file.write(step + "\n")
                filters[name].log_steps.clear()
                if filters[name].belief['open'] > 0.5:
                    print(f"Belief that {name} is open is high ({filters[name].belief['open']:.2f}). Passing through the door!")
                    log_file.write(f"  Passed through {name}.\n")
                    pass_through_door(pub)
                    break
                else:
                    print(f"Belief that {name} is open is low ({filters[name].belief['open']:.2f}). Waiting for new action/sensor...")
                    log_file.write(f"  Stayed at {name}.\n")
            log_file.write("\n")
    print(f"\nLog saved to {log_path}")

if __name__ == "__main__":
    main()