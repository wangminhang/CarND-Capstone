#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped,  Quaternion, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
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

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # For mocking the traffic light status.
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_light_mock_cb, queue_size=1)
        self.mock_traffic_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.obstacle_waypoints = None
        self.next_traffic_wp_idx = None
        self.current_velocity = None

        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)


        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb,
                         queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        rospy.Subscriber('/current_velocity', TwistStamped,
                         self.current_velocity_cb)

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

    def get_closest_waypoint(self, pose, waypoints):
        closest_wp_idx = None
        closest_wp_dist = 10 ** 10  # some really big distance.
        yaw = self.get_yaw(pose.orientation)

        # TODO: Is this necessary?
        #if not self.base_waypoints or len(self.base_waypoints) == 0:
        #    return closest_wp_idx, closest_wp_dist

        # Compute the waypoints ahead of the current_pose
        base_wp_len = len(waypoints)
        for i in range(base_wp_len):
            wp_pos = waypoints[i].pose.pose.position
            wp_x, wp_y, _ = self.convert_coord(
                (pose.position.x, pose.position.y, yaw),
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
            # Make sure we've got valid data before publishing
            if not self.pose or not self.base_waypoints or not self.next_traffic_wp_idx:
                rospy.logwarn("waiting for all data..")
                continue

            closest_wp_idx, closest_wp_dist = self.get_closest_waypoint(
                self.pose, self.base_waypoints)
            if not closest_wp_idx:
                continue

            # Note: it's not necessary to set the header info like stamp, seq, frame_id, etc.
            lane = Lane()

            base_wp_len = len(self.base_waypoints)

            # vel_inc = self.current_velocity / n_waypoints_to_light
            rospy.logwarn("---------------:  %s", self.current_velocity)
            vel = self.current_velocity
            for i in range(LOOKAHEAD_WPS):
                this_wp_idx = (closest_wp_idx + i) % base_wp_len
                next_wp_idx = (closest_wp_idx + i + 1) % base_wp_len
                n_waypoints_to_light = self.next_traffic_wp_idx - this_wp_idx
                dist_to_light = self.distance(self.base_waypoints, this_wp_idx,
                                              self.next_traffic_wp_idx)

                if vel <= 0:
                    time_until_light = 10 ** 10  # Big number
                else:
                    time_until_light = dist_to_light / vel

                if n_waypoints_to_light == 0:
                    vel = 0

                # Light is ahead of this point.
                elif n_waypoints_to_light >= 0:
                    if time_until_light < 5:
                        # should we start breaking yet?
                        dist_to_next = self.distance(self.base_waypoints,
                                                     this_wp_idx, next_wp_idx)
                        time_until_next = dist_to_next / vel
                        accel_required = dist_to_light / (
                        time_until_light ** 2)

                        vel -= accel_required * time_until_next
                    else:
                        vel = min(vel + 1, 50)  # TODO: Don't Hardcode.
                else:
                    vel = min(vel + 1, 50)  # TODO: Don't Hardcode.

                rospy.logwarn("vel:  %s", vel)
                # rospy.logwarn("n_waypoints_to_light:  %s", n_waypoints_to_light)

                self.set_waypoint_velocity(self.base_waypoints, next_wp_idx,
                                           vel)
                lane.waypoints.append(self.base_waypoints[next_wp_idx])

            # publish the waypoints
            self.final_waypoints_pub.publish(lane)

    def traffic_light_mock_cb(self, msg):
        if self.base_waypoints:
            i, dist = self.get_closest_waypoint(self.pose, msg.lights)
            #rospy.logwarn("nearest tl: %s", msg.lights[i].pose.pose)

            # TODO: May need to get the traffic line and not just the wp closest to the light?
            # config_string = rospy.get_param("/traffic_light_config")
            # self.config = yaml.load(config_string)

            nearest_wp_idx, dist = self.get_closest_waypoint(msg.lights[i].pose.pose, self.base_waypoints)

            #rospy.logwarn("nearest tl: %s", nearest_wp_idx)
            self.mock_traffic_light_pub.publish(nearest_wp_idx)

    def current_velocity_cb(self, msg):
        # rospy.logerr(msg)
        self.current_velocity = msg.twist.linear.x

    def pose_cb(self, msg):
        self.pose = msg.pose



    def waypoints_cb(self, msg):
        rospy.logwarn("waypoints_cb N:  %s", len(msg.waypoints))
        # NOTE: This should only happen once.
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.next_traffic_wp_idx = msg.data
        #rospy.logwarn("traffic wp:  %s", msg)

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
