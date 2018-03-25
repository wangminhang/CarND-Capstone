#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped,  Quaternion, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
from std_msgs.msg import Int32, Header
import os

import math
import copy
import numpy as np

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

DEBUG = os.getenv('DEBUG', False)
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
ONE_MPH = 0.44704
ONE_KPH = 1000. / 60. / 60.
MIN_VEL = 2.8 * ONE_KPH

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Subscribe to topics
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.obstacle_waypoints = None
        self.next_traffic_wp_idx = None
        self.last_traffic_wp_idx = -1
        self.current_wp_idx = None
        self.current_velocity = None

        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)

        self.max_vel = rospy.get_param('/waypoint_loader/velocity', 40.) * ONE_KPH

        # rospy.logerr('{} {}'.format(MIN_VEL, self.max_vel))

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

    def get_closest_waypoint(self, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        a = (x, y, z)

        d = lambda a, b: math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

        closest_wp_idx = None
        closest_wp_dist = 10 ** 10  # some really big distance.

        for i, wp in enumerate(self.base_waypoints):
            b = (wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z)
            dist = d(a, b)

            if dist < closest_wp_dist:
                closest_wp_dist = dist
                closest_wp_idx = i

        return closest_wp_idx

    def loop(self):
        """
        Loop to publish the waypoints and recommended speed
        :return: None
        """
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            if not self.base_waypoints or not self.pose:
                continue

            # get index of closest wp
            index = self.get_closest_waypoint(self.pose)
            index_end = min(index + LOOKAHEAD_WPS, len(self.base_waypoints))
            self.current_wp_idx = index

            # check if index wp is in front of the car
            ref_yaw = self.get_yaw(self.pose.pose.orientation)
            ref_x = self.pose.pose.position.x
            ref_y = self.pose.pose.position.y

            wp = self.base_waypoints[index]
            wp_x = wp.pose.pose.position.x
            wp_y = wp.pose.pose.position.y

            new_x, new_y, angle = self.convert_coord(ref_x, ref_y, ref_yaw, wp_x, wp_y)

            if new_x < 0 or new_y < 0:
                index += 1

            # check orientation
            if angle > math.pi / 4:
                index += 1
                if index > len(self.base_waypoints):
                    index = 0

            msg_pub = Lane()
            msg_pub.header.frame_id = 'waypoints_ahead'
            msg_pub.header.stamp = rospy.Time.now()
            msg_pub.waypoints = self.base_waypoints[index:index_end]

            self.final_waypoints_pub.publish(msg_pub)

            rate.sleep()

    def current_velocity_cb(self, msg):
        # rospy.logerr(msg)
        self.current_velocity = msg.twist.linear.x
        self.current_velocity = 0. if self.current_velocity < .1 else self.current_velocity

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        # rospy.logwarn("waypoints_cb N:  %s", len(msg.waypoints))
        # NOTE: This should only happen once.
        waypoints = msg.waypoints

        # adjust max velocity
        for i, wp in enumerate(waypoints):
            if self.get_waypoint_velocity(wp) > self.max_vel:
                self.set_waypoint_velocity(waypoints, i, self.max_vel)

        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # rospy.logwarn("traffic wp:  %s", msg)
        self.next_traffic_wp_idx = msg.data

        # check if it's a new index point
        if self.last_traffic_wp_idx == self.next_traffic_wp_idx or \
            not self.current_wp_idx or \
            not self.base_waypoints:

            return

        should_brake = True
        current_velocity = self.current_velocity

        if current_velocity <= 0.: current_velocity = MIN_VEL

        if self.next_traffic_wp_idx < 0:
            self.next_traffic_wp_idx = min(len(self.base_waypoints), self.current_wp_idx + LOOKAHEAD_WPS)
            should_brake = False
            distance = self.distance(self.base_waypoints, self.current_wp_idx, self.next_traffic_wp_idx)
        else:
            distance = self.distance(self.base_waypoints, self.current_wp_idx, self.next_traffic_wp_idx)

            # check time to brake with decel_limit
            if distance / current_velocity > abs(current_velocity / (self.decel_limit + .5)) and current_velocity > MIN_VEL:
                return

        # update last index to avoid multiple calls
        self.last_traffic_wp_idx = -1 if not should_brake else self.next_traffic_wp_idx

        # compute linear accel/decel
        local_max_vel = self.get_waypoint_velocity(self.base_waypoints[self.next_traffic_wp_idx])

        if not should_brake and current_velocity >= self.max_vel:
            return

        linear_accel = local_max_vel * self.accel_limit if not should_brake else current_velocity * abs(self.decel_limit)

        # update waypoints
        for i in range(self.current_wp_idx, self.next_traffic_wp_idx+1):
            # compute partial velocity until stop
            delta_dist = self.distance(self.base_waypoints, self.current_wp_idx, i)
            delta_vel = float(int(delta_dist / distance + 1)) * linear_accel

            if should_brake:
                # brake
                vel = max(current_velocity - delta_vel, 0.)
            else:
                # accelerate
                vel = max(min(current_velocity + delta_vel, self.max_vel), MIN_VEL)

            self.set_waypoint_velocity(self.base_waypoints, i, vel)

        if should_brake:
            rospy.logwarn("Red Light Detected")
        else:
            rospy.logwarn("Green Light Detected")

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

    def convert_coord(self, ref_x, ref_y, ref_yaw, pt_x, pt_y):
        shift_x = pt_x - ref_x
        shift_y = pt_y - ref_y
        cos_yaw = math.cos(ref_yaw)
        sin_yaw = math.sin(ref_yaw)

        new_x = cos_yaw * shift_x + sin_yaw * shift_y
        new_y = -sin_yaw * shift_x + cos_yaw * shift_y

        angle_from_ref_pt = math.atan2(new_x, new_y)

        return new_x, new_y, angle_from_ref_pt


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')