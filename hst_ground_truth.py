#!/usr/bin/python

# Author Harry Terkanian
# July 30, 2017

#hst_ground_truth.py
"""Beaverworks 2017 Summer Program, Week 3, Day 1, Lab Exercise 1"""

import rospy
import tf
from gazebo_msgs.msg import ModelStates

class HSTGroundTruthNode:

    def __init__( self ):

        #=====state variables===================================
        self.world_frames = []          # list of world_frames
        self.first_time   = True


        #=====ROS subscriptions=================================
        rospy.Subscriber( "/gazebo/model_states", ModelStates, self.model_state_callback )

        #=====ROS tf broadcaster================================
        self.broadcaster = tf.TransformBroadcaster()

    def model_state_callback( self, state_msg ):
        self.state_msg = state_msg

        if self.first_time:
            for state in self.state_msg.name:
                self.world_frames.append( state )
            print( self.world_frames )
            self.first_time = False

        print( self.state_msg.pose[2] )
        self.broadcastTF( self.state_msg.pose[2] )
            
    def broadcastTF( self, msg ):
        self.broadcaster.sendTransform( ( msg.position.x, msg.position.y, 0 ), ( msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w ), rospy.Time(), "racecar", "world" )


if __name__ == "__main__":
    rospy.init_node( "hst_ground_truth", anonymous = True )
    node = HSTGroundTruthNode()
    rospy.spin()
