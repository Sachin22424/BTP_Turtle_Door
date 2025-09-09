#!/usr/bin/env python3

import rospy
import turtlesim.srv
import turtlesim.msg
import geometry_msgs.msg
import std_srvs.srv
import math
import time
import random
import os

class Door:
    def __init__(self, name, x, y, turtle_name, true_state=None, push_success_prob=0.7):
        self.name = name
        self.x = x
        self.y = y
        self.turtle_name = turtle_name
        self.true_state = true_state if true_state else random.choice(["open", "closed"])
        self.push_success_prob = push_success_prob
        
    def spawn(self):
        try:
            spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
            spawner(self.x, self.y, 0, self.turtle_name)
            time.sleep(0.5)  # Wait for turtle to spawn
            
            # Set door color based on its state for better visualization
            set_pen = rospy.ServiceProxy(f'/{self.turtle_name}/set_pen', turtlesim.srv.SetPen)
            if self.true_state == "open":
                set_pen(0, 255, 0, 5, 0)  # Green for open doors
            else:
                set_pen(255, 0, 255, 5, 0)  # Magenta for closed doors
            
            rospy.loginfo(f"Spawned door {self.name} at ({self.x}, {self.y}) - True state: {self.true_state}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn service call failed for {self.name}: {e}")
    
    def simulate_push(self):
        """Simulate pushing the door"""
        if self.true_state == "open":
            return "open"
        
        if random.random() < self.push_success_prob:
            self.true_state = "open"
            return "open"
        else:
            return "closed"

class Robot:
    def __init__(self, robot_id, turtle_name, x, y, doors, threshold=0.5, comm_weight=0.3):
        self.robot_id = robot_id
        self.turtle_name = turtle_name
        self.x = x
        self.y = y
        self.doors = doors
        self.beliefs = {door.name: {"open": 0.5, "closed": 0.5} for door in doors}
        self.threshold = threshold
        self.doors_passed = []
        self.comm_weight = comm_weight
        self.log_entries = []
        
        # Wait for turtle to be spawned by main
        time.sleep(1)
        self.velocity_publisher = rospy.Publisher(f'/{turtle_name}/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber(f'/{turtle_name}/pose', turtlesim.msg.Pose, self.update_pose)
        self.pose = None
        
        # Wait for the pose to be updated
        timeout = 10
        start_time = time.time()
        while self.pose is None and (time.time() - start_time) < timeout:
            time.sleep(0.1)
            
        if self.pose is None:
            rospy.logerr(f"Failed to get pose for {self.turtle_name}")
    
    def update_pose(self, data):
        """Callback function to update the turtle's position"""
        self.pose = data
        self.x = data.x
        self.y = data.y
    
    def set_color(self, r, g, b):
        """Set the robot's color"""
        try:
            set_pen = rospy.ServiceProxy(f'/{self.turtle_name}/set_pen', turtlesim.srv.SetPen)
            set_pen(r, g, b, 3, 0)
            rospy.loginfo(f"Set {self.turtle_name} color to RGB({r}, {g}, {b})")
        except rospy.ServiceException as e:
            rospy.logerr(f"SetPen service call failed for {self.turtle_name}: {e}")
    
    def set_pen_thickness(self, thickness):
        """Set the robot's pen thickness for better visualization"""
        try:
            set_pen = rospy.ServiceProxy(f'/{self.turtle_name}/set_pen', turtlesim.srv.SetPen)
            # Get current color settings, just change thickness
            if self.robot_id == 1:
                set_pen(255, 0, 0, thickness, 0)  # Red with variable thickness
            else:
                set_pen(0, 255, 0, thickness, 0)  # Green with variable thickness
        except rospy.ServiceException as e:
            rospy.logerr(f"SetPen thickness service call failed for {self.turtle_name}: {e}")
    
    def move_to_door(self, door):
        """Move the robot to a specified door"""
        rospy.loginfo(f"{self.turtle_name} moving to {door.name} at ({door.x}, {door.y})")
        self.log_entries.append(f"Moving to {door.name} at ({door.x:.1f}, {door.y:.1f})")
        
        if self.pose is None:
            rospy.logerr(f"No pose available for {self.turtle_name}")
            return
        
        # Calculate distance and angle to the door
        distance = math.sqrt((door.x - self.x) ** 2 + (door.y - self.y) ** 2)
        angle = math.atan2(door.y - self.y, door.x - self.x) - self.pose.theta
        
        # Rotate to face the door
        self.rotate(angle)
        
        # Move to the door
        self.move(distance)
    
    def rotate(self, angle):
        """Rotate the robot by a certain angle"""
        vel_msg = geometry_msgs.msg.Twist()
        angular_speed = 2.0
        
        # Normalize angle
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        
        vel_msg.linear.x = 0
        vel_msg.angular.z = angular_speed if angle > 0 else -angular_speed
        
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        
        while current_angle < abs(angle) and not rospy.is_shutdown():
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
            
        # Stop rotation
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
    
    def move(self, distance):
        """Move the robot forward by a certain distance"""
        vel_msg = geometry_msgs.msg.Twist()
        speed = 2.0
        
        vel_msg.linear.x = speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        
        while current_distance < distance and not rospy.is_shutdown():
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = speed * (t1 - t0)
            
        # Stop moving
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)
    
    def update_belief_bayes(self, door, action, observation):
        """Update belief using Bayes filter with detailed calculations"""
        door_name = door.name
        
        # Prior beliefs bel(x_{t-1})
        bel_open_prev = self.beliefs[door_name]["open"]
        bel_closed_prev = self.beliefs[door_name]["closed"]
        
        self.log_entries.append(f"\n=== BAYES FILTER UPDATE FOR {door_name.upper()} ===")
        self.log_entries.append(f"ACTION: {action}, OBSERVATION: {observation}")
        self.log_entries.append(f"\nStep 1: Prior belief bel(x_{{t-1}})")
        self.log_entries.append(f"  bel(open) = {bel_open_prev:.4f}")
        self.log_entries.append(f"  bel(closed) = {bel_closed_prev:.4f}")
        self.log_entries.append(f"  Verification: {bel_open_prev:.4f} + {bel_closed_prev:.4f} = {bel_open_prev + bel_closed_prev:.4f}")
        
        # PREDICTION STEP: bel^-(x_t) = ∑ P(x_t | u_t, x_{t-1}) * bel(x_{t-1})
        self.log_entries.append(f"\nStep 2: PREDICTION STEP - Motion Model P(x_t | u_t, x_{{t-1}})")
        
        if action == "push":
            # Motion model probabilities for push action
            self.log_entries.append(f"  Motion Model for action 'push':")
            self.log_entries.append(f"    P(open | push, closed) = 0.8")
            self.log_entries.append(f"    P(closed | push, closed) = 0.2") 
            self.log_entries.append(f"    P(open | push, open) = 1.0")
            self.log_entries.append(f"    P(closed | push, open) = 0.0")
            
            # Detailed calculation
            term1_open = 0.8 * bel_closed_prev
            term2_open = 1.0 * bel_open_prev
            term1_closed = 0.2 * bel_closed_prev
            term2_closed = 0.0 * bel_open_prev
            
            self.log_entries.append(f"\n  Prediction calculations:")
            self.log_entries.append(f"    bel^-(open) = P(open|push,closed)*bel(closed) + P(open|push,open)*bel(open)")
            self.log_entries.append(f"                = 0.8 × {bel_closed_prev:.4f} + 1.0 × {bel_open_prev:.4f}")
            self.log_entries.append(f"                = {term1_open:.4f} + {term2_open:.4f} = {term1_open + term2_open:.4f}")
            
            self.log_entries.append(f"    bel^-(closed) = P(closed|push,closed)*bel(closed) + P(closed|push,open)*bel(open)")
            self.log_entries.append(f"                  = 0.2 × {bel_closed_prev:.4f} + 0.0 × {bel_open_prev:.4f}")
            self.log_entries.append(f"                  = {term1_closed:.4f} + {term2_closed:.4f} = {term1_closed + term2_closed:.4f}")
            
            bel_open = term1_open + term2_open
            bel_closed = term1_closed + term2_closed
            
        else:  # do nothing
            self.log_entries.append(f"  Motion Model for action 'do nothing':")
            self.log_entries.append(f"    P(open | do_nothing, open) = 1.0")
            self.log_entries.append(f"    P(closed | do_nothing, open) = 0.0")
            self.log_entries.append(f"    P(open | do_nothing, closed) = 0.0")
            self.log_entries.append(f"    P(closed | do_nothing, closed) = 1.0")
            
            term1_open = 0.0 * bel_closed_prev
            term2_open = 1.0 * bel_open_prev
            term1_closed = 1.0 * bel_closed_prev
            term2_closed = 0.0 * bel_open_prev
            
            self.log_entries.append(f"\n  Prediction calculations:")
            self.log_entries.append(f"    bel^-(open) = 0.0 × {bel_closed_prev:.4f} + 1.0 × {bel_open_prev:.4f} = {term1_open + term2_open:.4f}")
            self.log_entries.append(f"    bel^-(closed) = 1.0 × {bel_closed_prev:.4f} + 0.0 × {bel_open_prev:.4f} = {term1_closed + term2_closed:.4f}")
            
            bel_open = term1_open + term2_open
            bel_closed = term1_closed + term2_closed
        
        # Check if normalization needed after prediction
        total_before_norm = bel_open + bel_closed
        self.log_entries.append(f"\n  Prediction result (before normalization):")
        self.log_entries.append(f"    bel^-(open) = {bel_open:.4f}")
        self.log_entries.append(f"    bel^-(closed) = {bel_closed:.4f}")
        self.log_entries.append(f"    Sum = {total_before_norm:.4f}")
        
        if abs(total_before_norm - 1.0) > 1e-10:
            eta_pred = 1.0 / total_before_norm
            bel_open_unnorm = bel_open
            bel_closed_unnorm = bel_closed
            bel_open *= eta_pred
            bel_closed *= eta_pred
            self.log_entries.append(f"\n  Normalization needed: η = 1/{total_before_norm:.4f} = {eta_pred:.4f}")
            self.log_entries.append(f"    Normalized bel^-(open) = {bel_open_unnorm:.4f} × {eta_pred:.4f} = {bel_open:.4f}")
            self.log_entries.append(f"    Normalized bel^-(closed) = {bel_closed_unnorm:.4f} × {eta_pred:.4f} = {bel_closed:.4f}")
        else:
            self.log_entries.append(f"  No normalization needed (sum ≈ 1.0)")
        
        # CORRECTION STEP: bel(x_t) = η * P(z_t | x_t) * bel^-(x_t)
        self.log_entries.append(f"\nStep 3: CORRECTION STEP - Sensor Model P(z_t | x_t)")
        self.log_entries.append(f"  Sensor Model probabilities:")
        self.log_entries.append(f"    P(observe_open | door_open) = 0.6")
        self.log_entries.append(f"    P(observe_closed | door_open) = 0.4")
        self.log_entries.append(f"    P(observe_open | door_closed) = 0.2")
        self.log_entries.append(f"    P(observe_closed | door_closed) = 0.8")
        
        bel_open_before_correction = bel_open
        bel_closed_before_correction = bel_closed
        
        if observation == "open":
            likelihood_open = 0.6
            likelihood_closed = 0.2
            self.log_entries.append(f"\n  Observed: {observation}")
            self.log_entries.append(f"    P(observe_open | door_open) = {likelihood_open}")
            self.log_entries.append(f"    P(observe_open | door_closed) = {likelihood_closed}")
            
            bel_open *= likelihood_open
            bel_closed *= likelihood_closed
            
            self.log_entries.append(f"\n  Correction calculations (before normalization):")
            self.log_entries.append(f"    bel(open) ∝ P(open|obs) × bel^-(open) = {likelihood_open} × {bel_open_before_correction:.4f} = {bel_open:.4f}")
            self.log_entries.append(f"    bel(closed) ∝ P(open|closed) × bel^-(closed) = {likelihood_closed} × {bel_closed_before_correction:.4f} = {bel_closed:.4f}")
            
        else:  # observation = "closed"
            likelihood_open = 0.4
            likelihood_closed = 0.8
            self.log_entries.append(f"\n  Observed: {observation}")
            self.log_entries.append(f"    P(observe_closed | door_open) = {likelihood_open}")
            self.log_entries.append(f"    P(observe_closed | door_closed) = {likelihood_closed}")
            
            bel_open *= likelihood_open
            bel_closed *= likelihood_closed
            
            self.log_entries.append(f"\n  Correction calculations (before normalization):")
            self.log_entries.append(f"    bel(open) ∝ P(closed|open) × bel^-(open) = {likelihood_open} × {bel_open_before_correction:.4f} = {bel_open:.4f}")
            self.log_entries.append(f"    bel(closed) ∝ P(closed|closed) × bel^-(closed) = {likelihood_closed} × {bel_closed_before_correction:.4f} = {bel_closed:.4f}")
        
        # Normalization step for correction
        total_unnorm = bel_open + bel_closed
        eta_corr = 1.0 / total_unnorm
        bel_open_unnorm = bel_open
        bel_closed_unnorm = bel_closed
        bel_open *= eta_corr
        bel_closed *= eta_corr
        
        self.log_entries.append(f"\n  Final Normalization:")
        self.log_entries.append(f"    Sum before normalization = {bel_open_unnorm:.4f} + {bel_closed_unnorm:.4f} = {total_unnorm:.4f}")
        self.log_entries.append(f"    Normalization constant η = 1/{total_unnorm:.4f} = {eta_corr:.4f}")
        self.log_entries.append(f"    Final bel(open) = {bel_open_unnorm:.4f} × {eta_corr:.4f} = {bel_open:.4f}")
        self.log_entries.append(f"    Final bel(closed) = {bel_closed_unnorm:.4f} × {eta_corr:.4f} = {bel_closed:.4f}")
        self.log_entries.append(f"    Verification: {bel_open:.4f} + {bel_closed:.4f} = {bel_open + bel_closed:.4f}")
        
        # Why this normalization?
        self.log_entries.append(f"\n  WHY NORMALIZE?")
        self.log_entries.append(f"    - Bayes rule gives us P(x|z) ∝ P(z|x) × P(x)")
        self.log_entries.append(f"    - The proportionality constant ensures ∑P(x|z) = 1")
        self.log_entries.append(f"    - This maintains the probability distribution property")
        self.log_entries.append(f"    - Alternative: Maximum likelihood would pick max, losing uncertainty info")
        self.log_entries.append(f"    - Alternative: Additive update would violate probability axioms")
        
        # Update beliefs
        self.beliefs[door_name]["open"] = bel_open
        self.beliefs[door_name]["closed"] = bel_closed
        
        self.log_entries.append(f"\n=== BAYES FILTER COMPLETE FOR {door_name.upper()} ===")
    
    def merge_beliefs(self, door, other_belief):
        """Merge this robot's belief with another robot's belief using weighted average"""
        door_name = door.name
        
        before_open = self.beliefs[door_name]["open"]
        before_closed = self.beliefs[door_name]["closed"]
        
        self.log_entries.append(f"\n=== BELIEF MERGING FOR {door_name.upper()} ===")
        self.log_entries.append(f"COMMUNICATION: Receiving belief from other robot")
        self.log_entries.append(f"\n  Current robot's belief:")
        self.log_entries.append(f"    bel_self(open) = {before_open:.4f}")
        self.log_entries.append(f"    bel_self(closed) = {before_closed:.4f}")
        self.log_entries.append(f"\n  Other robot's belief:")
        self.log_entries.append(f"    bel_other(open) = {other_belief['open']:.4f}")
        self.log_entries.append(f"    bel_other(closed) = {other_belief['closed']:.4f}")
        
        # Weighted average merge
        self_weight = 1 - self.comm_weight
        other_weight = self.comm_weight
        
        self.log_entries.append(f"\n  WEIGHTED CONSENSUS FUSION:")
        self.log_entries.append(f"    Communication weight (other robot's influence) = {self.comm_weight:.2f}")
        self.log_entries.append(f"    Self weight = 1 - {self.comm_weight:.2f} = {self_weight:.2f}")
        self.log_entries.append(f"    Other weight = {other_weight:.2f}")
        
        # Calculate merged beliefs step by step
        merged_open_term1 = self_weight * self.beliefs[door_name]["open"]
        merged_open_term2 = other_weight * other_belief["open"]
        merged_closed_term1 = self_weight * self.beliefs[door_name]["closed"]
        merged_closed_term2 = other_weight * other_belief["closed"]
        
        merged_open = merged_open_term1 + merged_open_term2
        merged_closed = merged_closed_term1 + merged_closed_term2
        
        self.log_entries.append(f"\n  Fusion calculations:")
        self.log_entries.append(f"    merged(open) = w_self × bel_self(open) + w_other × bel_other(open)")
        self.log_entries.append(f"                 = {self_weight:.2f} × {before_open:.4f} + {other_weight:.2f} × {other_belief['open']:.4f}")
        self.log_entries.append(f"                 = {merged_open_term1:.4f} + {merged_open_term2:.4f} = {merged_open:.4f}")
        
        self.log_entries.append(f"    merged(closed) = w_self × bel_self(closed) + w_other × bel_other(closed)")
        self.log_entries.append(f"                   = {self_weight:.2f} × {before_closed:.4f} + {other_weight:.2f} × {other_belief['closed']:.4f}")
        self.log_entries.append(f"                   = {merged_closed_term1:.4f} + {merged_closed_term2:.4f} = {merged_closed:.4f}")
        
        # Normalize (should be ≈1 already for weighted average, but ensure exact)
        total = merged_open + merged_closed
        self.log_entries.append(f"\n  Normalization check:")
        self.log_entries.append(f"    Sum before normalization = {merged_open:.4f} + {merged_closed:.4f} = {total:.4f}")
        
        if abs(total - 1.0) > 1e-10:
            eta = 1.0 / total
            merged_open_unnorm = merged_open
            merged_closed_unnorm = merged_closed
            merged_open *= eta
            merged_closed *= eta
            self.log_entries.append(f"    Normalization needed: η = 1/{total:.4f} = {eta:.4f}")
            self.log_entries.append(f"    Normalized merged(open) = {merged_open_unnorm:.4f} × {eta:.4f} = {merged_open:.4f}")
            self.log_entries.append(f"    Normalized merged(closed) = {merged_closed_unnorm:.4f} × {eta:.4f} = {merged_closed:.4f}")
        else:
            self.log_entries.append(f"    No normalization needed (sum ≈ 1.0)")
        
        # Update beliefs
        self.beliefs[door_name]["open"] = merged_open
        self.beliefs[door_name]["closed"] = merged_closed
        
        # Show the change
        change_open = merged_open - before_open
        change_closed = merged_closed - before_closed
        
        self.log_entries.append(f"\n  FUSION RESULT:")
        self.log_entries.append(f"    Final merged(open) = {merged_open:.4f}")
        self.log_entries.append(f"    Final merged(closed) = {merged_closed:.4f}")
        self.log_entries.append(f"    Change in open belief: {change_open:+.4f}")
        self.log_entries.append(f"    Change in closed belief: {change_closed:+.4f}")
        
        # Explain why weighted average
        self.log_entries.append(f"\n  WHY WEIGHTED AVERAGE?")
        self.log_entries.append(f"    - Preserves uncertainty from both robots")
        self.log_entries.append(f"    - Communication weight controls trust level")
        self.log_entries.append(f"    - Alternative: Multiplication would assume independence")
        self.log_entries.append(f"    - Alternative: Max would lose one robot's information")
        self.log_entries.append(f"    - This method: Consensus-based distributed estimation")
        
        self.log_entries.append(f"\n=== BELIEF MERGING COMPLETE FOR {door_name.upper()} ===")
    
    def decide_action(self, door):
        """Decide whether to push or do nothing based on belief"""
        if self.beliefs[door.name]["open"] < self.threshold:
            return "push"
        else:
            return "do nothing"
    
    def can_pass_all_doors(self):
        """Check if all doors pass the threshold"""
        return all(self.beliefs[door.name]["open"] > self.threshold for door in self.doors)
    
    def save_log(self, filename):
        """Save the log entries to a file"""
        with open(filename, 'a') as f:
            f.write(f"\n--- {self.turtle_name} Log ---\n")
            for entry in self.log_entries:
                f.write(f"{self.turtle_name}: {entry}\n")
        self.log_entries = []
    
    def find_closest_door(self):
        """Find the closest unvisited door"""
        unvisited_doors = [door for door in self.doors if door.name not in self.doors_passed]
        if not unvisited_doors:
            return None
        
        min_distance = float('inf')
        closest_door = None
        
        for door in unvisited_doors:
            distance = math.sqrt((door.x - self.x) ** 2 + (door.y - self.y) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_door = door
        
        return closest_door
    
    def move_to_door_smooth(self, door):
        """Move the robot to a specified door with smoother movement"""
        rospy.loginfo(f"{self.turtle_name} moving to {door.name} at ({door.x}, {door.y})")
        self.log_entries.append(f"Moving to {door.name} at ({door.x:.1f}, {door.y:.1f})")
        
        if self.pose is None:
            rospy.logerr(f"No pose available for {self.turtle_name}")
            return
        
        # Calculate distance to the door
        target_distance = 0.5  # Stop 0.5 units away from the door
        
        while True:
            # Update current distance and angle to the door
            dx = door.x - self.x
            dy = door.y - self.y
            distance = math.sqrt(dx ** 2 + dy ** 2)
            
            if distance <= target_distance:
                break
            
            # Calculate angle to the door
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.pose.theta
            
            # Normalize angle difference
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            
            vel_msg = geometry_msgs.msg.Twist()
            
            # Smooth angular movement
            if abs(angle_diff) > 0.1:
                vel_msg.angular.z = 1.5 * (1 if angle_diff > 0 else -1)
                vel_msg.linear.x = 0.5  # Move slowly while turning
            else:
                vel_msg.angular.z = 0
                vel_msg.linear.x = min(1.5, distance)  # Slow down as we approach
            
            self.velocity_publisher.publish(vel_msg)
            time.sleep(0.1)
        
        # Stop the robot
        vel_msg = geometry_msgs.msg.Twist()
        self.velocity_publisher.publish(vel_msg)
        
        rospy.loginfo(f"{self.turtle_name} reached {door.name}")

def main():
    rospy.init_node('multi_robot_door_bayes', anonymous=True)
    rospy.loginfo("Starting Multi-Robot Door Bayes Filter")
    
    # Wait for turtlesim_node to start and spawn service to be available
    rospy.wait_for_service('spawn')
    rospy.loginfo("Spawn service is available")
    
    # Clear existing turtles
    try:
        clear = rospy.ServiceProxy('clear', std_srvs.srv.Empty)
        clear()
    except rospy.ServiceException as e:
        rospy.logwarn(f"Clear service call failed: {e}")
    
    # Create log file
    log_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'multi_robot_door_bayes_log.txt')
    with open(log_file, 'w') as f:
        f.write("Multi-Robot Door Bayes Filter Log\n")
        f.write("===============================\n\n")
    
    # Spawn robots first at opposite corners
    try:
        robot1_spawn = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        robot1_spawn(1.0, 1.0, 0, "robot1")  # Bottom left corner
        time.sleep(0.5)
        
        robot2_spawn = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        robot2_spawn(10.0, 10.0, 0, "robot2")  # Top right corner
        time.sleep(0.5)
        
        rospy.loginfo("Robots spawned successfully")
    except rospy.ServiceException as e:
        rospy.logerr(f"Robot spawn failed: {e}")
        return
    
    # Create and spawn doors at different corners and positions
    doors = []
    door_positions = [(3.0, 3.0), (8.0, 2.0), (2.0, 8.0), (9.0, 7.0), (5.5, 5.5)]  # Spread across the map
    
    for i, (x, y) in enumerate(door_positions):
        name = f"door{i+1}"
        door = Door(name, x, y, name)
        door.spawn()
        doors.append(door)
        time.sleep(0.3)  # Small delay between spawns
    
    rospy.loginfo("All doors spawned")
    
    # Create robot objects
    robot1 = Robot(1, "robot1", 1.0, 1.0, doors, threshold=0.6, comm_weight=0.3)
    robot2 = Robot(2, "robot2", 10.0, 10.0, doors, threshold=0.5, comm_weight=0.4)
    
    # Set robot colors (different colors for each robot)
    robot1.set_color(255, 0, 0)  # Red
    robot2.set_color(0, 255, 0)  # Green
    
    # Set pen thickness
    robot1.set_pen_thickness(3)
    robot2.set_pen_thickness(3)
    
    # Set pen thickness
    robot1.set_pen_thickness(3)
    robot2.set_pen_thickness(3)
    
    rospy.loginfo("Starting independent robot exploration")
    
    # Track robots that are still active
    active_robots = [robot1, robot2]
    robots_beliefs_shared = {robot1.robot_id: False, robot2.robot_id: False}
    
    while len(active_robots) > 0 and not rospy.is_shutdown():
        robots_to_remove = []
        
        for robot in active_robots:
            # Check if robot can pass all doors
            if robot.can_pass_all_doors():
                rospy.loginfo(f"{robot.turtle_name} can pass all doors. Stopping this robot.")
                robots_to_remove.append(robot)
                continue
            
            # Find closest unvisited door
            closest_door = robot.find_closest_door()
            if closest_door is None:
                rospy.loginfo(f"{robot.turtle_name} has visited all doors but cannot pass all. Stopping.")
                robots_to_remove.append(robot)
                continue
            
            rospy.loginfo(f"\n--- {robot.turtle_name} processing {closest_door.name} ---")
            
            # Move to the closest door using smooth movement
            robot.move_to_door_smooth(closest_door)
            
            # Robot decides action based on current belief
            action = robot.decide_action(closest_door)
            robot.log_entries.append(f"Decided to {action} {closest_door.name}")
            rospy.loginfo(f"{robot.turtle_name} decides to {action} {closest_door.name}")
            
            # Get observation based on action
            if action == "push":
                observation = closest_door.simulate_push()
                robot.log_entries.append(f"Pushed {closest_door.name} and observed it's {observation}")
            else:
                observation = closest_door.true_state
                robot.log_entries.append(f"Observed {closest_door.name} is {observation}")
            
            # Update robot's belief using Bayes filter
            robot.update_belief_bayes(closest_door, action, observation)
            
            # Check if robot can pass through this door
            if robot.beliefs[closest_door.name]["open"] > robot.threshold:
                robot.log_entries.append(f"Can pass through {closest_door.name}")
                robot.doors_passed.append(closest_door.name)
            else:
                robot.log_entries.append(f"Cannot pass through {closest_door.name} (belief: {robot.beliefs[closest_door.name]['open']:.3f})")
            
            robot.save_log(log_file)
            
            # Share beliefs with other active robots after visiting a door
            for other_robot in active_robots:
                if other_robot.robot_id != robot.robot_id:
                    other_robot.log_entries.append(f"Receiving belief update from {robot.turtle_name} for {closest_door.name}")
                    other_robot.merge_beliefs(closest_door, robot.beliefs[closest_door.name])
                    other_robot.save_log(log_file)
            
            time.sleep(0.5)  # Short pause between robot actions
        
        # Remove robots that have finished
        for robot in robots_to_remove:
            active_robots.remove(robot)
        
        # If both robots are still active, add a small delay to prevent race conditions
        if len(active_robots) > 1:
            time.sleep(0.3)
    
    # Final summary
    with open(log_file, 'a') as f:
        f.write("\n" + "="*50 + "\n")
        f.write("FINAL SUMMARY\n")
        f.write("="*50 + "\n")
        f.write(f"{robot1.turtle_name} passed through doors: {robot1.doors_passed}\n")
        f.write(f"{robot2.turtle_name} passed through doors: {robot2.doors_passed}\n")
        f.write(f"{robot1.turtle_name} can pass all doors: {robot1.can_pass_all_doors()}\n")
        f.write(f"{robot2.turtle_name} can pass all doors: {robot2.can_pass_all_doors()}\n")
        f.write("\nFinal beliefs for all doors:\n")
        for door in doors:
            f.write(f"{door.name}: {robot1.turtle_name} open={robot1.beliefs[door.name]['open']:.3f}, "
                   f"{robot2.turtle_name} open={robot2.beliefs[door.name]['open']:.3f}\n")
    
    rospy.loginfo("\nSummary:")
    rospy.loginfo(f"{robot1.turtle_name} passed through doors: {robot1.doors_passed}")
    rospy.loginfo(f"{robot2.turtle_name} passed through doors: {robot2.doors_passed}")
    rospy.loginfo(f"Log saved to: {log_file}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
