#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped,  Quaternion, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Header
import os

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

DEBUG = os.getenv('DEBUG', 1)
DEBUGGER_HOST = os.getenv('DEBUGGER_HOST', 'docker.for.mac.localhost')
DEBUGGER_PORT = int(os.getenv('DEBUGGER_PORT', '8989'))

if DEBUG:
    import pydevd
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    errno = s.connect_ex((DEBUGGER_HOST, DEBUGGER_PORT))
    s.close()
    if errno:
       print("Couldn't connect to the debugger at {}:{}, driving on.. "
             .format(DEBUGGER_HOST, DEBUGGER_PORT))
    else:
        pydevd.settrace(DEBUGGER_HOST, port=DEBUGGER_PORT, suspend=False,
                        stdoutToServer=False, stderrToServer=False)



LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.traffic_waypoints = None
        self.obstacle_waypoints = None

        self.loop()

    def get_yaw(self, orientation):
        """
        Compute yaw from orientation, which is in Quaternion.
        """
        # orientation = msg.pose.orientation
        euler = tf.transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w])
        yaw = euler[2]
        return yaw

    def get_closest_waypoint(self):
        closest_wp_idx = None
        closest_wp_dist = 10 ** 10  # some really big distance.
        yaw = self.get_yaw(self.pose.orientation)

        # TODO: Is this necessary?
        #if not self.base_waypoints or len(self.base_waypoints) == 0:
        #    return closest_wp_idx, closest_wp_dist

        # Compute the waypoints ahead of the current_pose
        base_wp_len = len(self.base_waypoints)
        for i in range(base_wp_len):
            wp_pos = self.base_waypoints[i].pose.pose.position
            wp_x, wp_y, _ = self.convert_coord(
                (self.pose.position.x, self.pose.position.y, yaw),
                (wp_pos.x, wp_pos.y, 0)
            )
            dist = math.sqrt((wp_x** 2) +(wp_y** 2))
            if wp_x > 0 and dist < closest_wp_dist:
                closest_wp_dist = dist
                closest_wp_idx = i

        return closest_wp_idx, closest_wp_dist

    def loop(self):
        """
        Loop to publish the waypoints and recommended speed
        :return: None
        """
        # limit publishing rate (uses rate.sleep later)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.logwarn("loop pose: %s", self.pose)

            # TODO: Should we move the pose_cb logic back in here? It seems to
            # Run on it's own thread and we NEED to have something like
            # This loop, or the node will just end.
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg.pose

        # Make sure we've got valid data before publishing
        if not self.pose or not self.base_waypoints:
            rospy.logwarn("waiting for all data..")
            return

        closest_wp_idx, closest_wp_dist = self.get_closest_waypoint()
        if not closest_wp_idx:
            return

        # Note: it's not necessary to set the header info like stamp, seq, frame_id, etc.
        lane = Lane()
        base_wp_len = len(self.base_waypoints)
        for i in range(LOOKAHEAD_WPS):
            # for i in range(200):
            next_wp_idx = (closest_wp_idx + i) % base_wp_len
            lane.waypoints.append(self.base_waypoints[next_wp_idx])

        # publish the waypoints
        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, msg):
        rospy.logwarn("waypoints_cb N:  %s", len(msg.waypoints))
        # NOTE: This should only happen once.
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoints = msg

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.obstacle_waypoints = msg

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def convert_coord(self, (ref_x, ref_y, ref_yaw), (pt_x, pt_y, pt_yaw)):
        shift_x = pt_x - ref_x
        shift_y = pt_y - ref_y
        cos_yaw = math.cos(ref_yaw)
        sin_yaw = math.sin(ref_yaw)

        new_x = cos_yaw * shift_x + sin_yaw * shift_y
        new_y = -sin_yaw * shift_x + cos_yaw * shift_y
        #new_yaw = math.fmod(pt_yaw - ref_yaw, math.pi*2)

        angle_from_ref_pt = math.atan2(new_x, new_y)
        return new_x, new_y, angle_from_ref_pt


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
