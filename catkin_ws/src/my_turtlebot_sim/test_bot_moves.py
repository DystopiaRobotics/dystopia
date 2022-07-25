#!/usr/bin/env python

import math
import unittest
import rospy
from gazebo_msgs.msg import ModelStates


class TestBotMoves(unittest.TestCase):
    def test_movement(self):
        """Test that the robot is continuously movement by tracking how
        far it has travelled from it's original spawn point (0,0,0)
        """
        rospy.init_node('test_movement', log_level=rospy.DEBUG)

        self.idx = None
        self.x = 0.0
        self.y = 0.0

        # We're expecting the robot to move this many meters from spawn
        expected_travel = 1.5

        # Subscribe to Gazebo model states, so we can check if our robot moved.
        self.subscriber = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self.movement_callback)

        # Keep iterating until the robot travels as far as we expect
        travelled = self.get_distance_from_spawn()
        while travelled < expected_travel and not rospy.is_shutdown():
            rospy.loginfo('Travelled; %s m', round(travelled, 1))
            rospy.sleep(1)
            travelled = self.get_distance_from_spawn()

        # The robot has traveled as far as we expect. Test complete!
        assert travelled >= expected_travel

    def movement_callback(self, data):
        """Record the robots simulation world position whenever it's
        updated, so that we can check how far it has travelled from
        the spawn point.
        """

        # Find the index of the turtlebot model within all Gazebo models.
        if self.idx is None:
            self.idx = 0
            for name in data.name:
                if name == 'turtlebot3_burger':
                    break
                self.idx += 1

        # Save current X/Y position in the sim world
        self.x = data.pose[self.idx].position.x
        self.y = data.pose[self.idx].position.y

    def get_distance_from_spawn(self):
        """Use pythagoras to get total distance away from the spawn point
        """
        return math.sqrt(abs(self.x) ** 2 + abs(self.y) ** 2)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('my_turtlebot_sim', 'test_bot_moves', TestBotMoves)
