#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class DoorBayesFilter:
    def __init__(self):
        self.bel_open = 0.5
        self.bel_closed = 0.5

        # Sensor model - P(observation | true_state)
        self.sensor_model = {
            'open': {'open': 0.6, 'closed': 0.4},
            'closed': {'open': 0.2, 'closed': 0.8}
        }

        # Motion model - P(next_state | action, current_state)
        self.motion_model = {
            'do nothing': {
                'open': {'open': 1.0, 'closed': 0.0},
                'closed': {'open': 0.0, 'closed': 1.0}
            },
            'push': {
                'open': {'open': 1.0, 'closed': 0.0},
                'closed': {'open': 0.8, 'closed': 0.2}
            }
        }

        rospy.Subscriber("/door_action", String, self.action_callback)
        rospy.Subscriber("/door_sensor", String, self.sensor_callback)
        self.pub_belief = rospy.Publisher("/door_belief", String, queue_size=10)

    def action_callback(self, msg):
        self.last_action = msg.data
        rospy.loginfo(f"Received action: {self.last_action}")

    def sensor_callback(self, msg):
        if not hasattr(self, 'last_action'):
            rospy.logwarn("No action yet, skipping update.")
            return

        z = msg.data
        u = self.last_action
        rospy.loginfo(f"Received sensor: {z}")

        try:
            # Prediction step
            bel_open_pred = (
                self.motion_model[u]['open']['open'] * self.bel_open +
                self.motion_model[u]['closed']['open'] * self.bel_closed
            )
            bel_closed_pred = (
                self.motion_model[u]['open']['closed'] * self.bel_open +
                self.motion_model[u]['closed']['closed'] * self.bel_closed
            )

            # Correction step
            bel_open = self.sensor_model['open'][z] * bel_open_pred
            bel_closed = self.sensor_model['closed'][z] * bel_closed_pred

            # Normalize
            eta = 1.0 / (bel_open + bel_closed)
            self.bel_open = eta * bel_open
            self.bel_closed = eta * bel_closed

            rospy.loginfo(f"Updated belief: open={self.bel_open:.3f}, closed={self.bel_closed:.3f}")
            self.pub_belief.publish(f"open={self.bel_open:.3f}, closed={self.bel_closed:.3f}")
        
        except KeyError as e:
            rospy.logerr(f"Invalid key in sensor or motion model: {e}. Did you use 'closed' instead of 'close'?")

if __name__ == "__main__":
    rospy.init_node("bayes_filter_node")
    bf = DoorBayesFilter()
    rospy.spin()
