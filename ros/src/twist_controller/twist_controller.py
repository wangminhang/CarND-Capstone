import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):

        self.accel_limit = kwargs['accel_limit']

        # TODO: Implement
        pass

    def control(self, prop_x_vel, prop_ang_vel, curr_vel, curr_vel_last, curr_time):

        # Calculate the acceleration needed to go from the current velocity to
        # the proposed x velocity in the amount of time since the last update.
        time_delta = curr_time - curr_vel_last
        vel_delta = curr_vel - prop_x_vel
        accel = vel_delta / time_delta.to_sec()

        # TODO: This is the max acceleration, but how do we know how much
        # throttle to use, PID controller?
        max_accel = max(accel, self.accel_limit)
        rospy.logerr(accel)

        #TODO: How do we convert max_steer_angle to angular velocity?
        # I assume max_steer_angle is the steering wheel angle and not angle of the wheels?

        # Return throttle, brake, steer
        # TODO: These are just hardcoded for now until we know how to set the right values.
        return 1., 0., 0.
