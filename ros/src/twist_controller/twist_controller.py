import rospy
import time

from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_VEL = 10


class Controller(object):
    def __init__(self, *args, **kwargs):

        self.accel_limit = kwargs['accel_limit']
        self.last_t = None

        self.yaw_controller = YawController(kwargs['wheel_base'], kwargs['steer_ratio'],
                                         ONE_MPH, kwargs['max_lat_accel'],
                                         kwargs['max_steer_angle'])

        self.filter = LowPassFilter(0.2, 0.1)

        # TODO: Implement
        pass

    def control(self, target_linear_vel, target_angular_vel, curr_vel, dbw_enabled):

        # Calculate the acceleration needed to go from the current velocity to
        # the proposed x velocity in the amount of time since the last update.

        if self.last_t is None or not dbw_enabled:
            self.last_t = rospy.get_time()

        rospy.logerr('{} {}'.format(target_linear_vel.x, curr_vel.x))

        dt = rospy.get_time() - self.last_t
        dv = MAX_VEL * ONE_MPH - curr_vel.x

        throttle = dv / dt
        throttle = max(0., min(1., throttle))
        brake = 0.

        if dv < 0:
            brake = -5.0 * dv
            brake = max(1., brake)
            # throttle = 0.

        # time_delta = curr_time - curr_vel_last
        # vel_delta = prop_x_vel - curr_vel
        # accel = vel_delta / time_delta.to_sec()

        # TODO: This is the max acceleration, but how do we know how much
        # throttle to use, PID controller?
        # max_accel = min(accel, self.accel_limit)
        # rospy.logerr('{} {}'.format(vel_delta, time_delta.to_sec()))

        #TODO: How do we convert max_steer_angle to angular velocity?
        # I assume max_steer_angle is the steering wheel angle and not angle of the wheels?

        # Return throttle, brake, steer
        # TODO: These are just hardcoded for now until we know how to set the right values.
        # return 1., 0., 0.
        steer = self.yaw_controller.get_steering(target_linear_vel.x, target_angular_vel.z, curr_vel.x)
        steer = self.filter.filt(steer)

        self.last_t = time.time()

        return throttle, brake, steer