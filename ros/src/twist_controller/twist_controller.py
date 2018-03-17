import rospy
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
SPEED_LIMIT_MPS = 40.0 * ONE_MPH # Assume 40 mph is speed limit for now.


class Controller(object):
    def __init__(self, *args, **kwargs):

        self.accel_limit = kwargs['accel_limit']
        self.throttle_PID = PID(2, 0, 0) # TODO: optimize these weights on P, I, and D.
        self.last_stamp = None # ros timestamp of the last time control() was called.
        # TODO: Add steering PID controller.


    def control(self, target_x_vel, target_ang_vel, curr_x_vel, dbw_enabled):
        target_x_vel = SPEED_LIMIT_MPS

        if self.last_stamp is None or dbw_enabled:
            self.last_stamp = rospy.get_time() # On the first pass, just initialize last_stamp.
            # Reset the PID Controllers just in case.
            self.throttle_PID.reset()
            return 0.0, 0.0, 0.0

        # Note: It might be better to use the timestamps in the messages for better accuracy?
        time_delta = rospy.get_time() - self.last_stamp

        # Note: vel_err is positive when car is too slow and negative when going too fast.
        vel_err = target_x_vel - curr_x_vel
        # rospy.logerr("vel_err %s", vel_err)
        throttle = self.throttle_PID.step(vel_err, time_delta)
        # rospy.logerr("throttle %s", throttle)

        # Make sure the returned throttle is within limits.
        throttle = max(0.0, min(1.0, throttle))

        # TODO: Do we care about the acceleration and jerk here?
        if throttle < 0:
            brake = -throttle
            throttle = 0

        # NOTE: Using the current P: 2, I:0, and D:0, the car breaks a bit slowly.
        # We may want to set independent break and steering, or just multiply by some factor.
        # Velocity based waypoints may make this less necessary.
        else:
            brake = 0

        #TODO: How do we convert max_steer_angle to angular velocity?
        # I assume max_steer_angle is the steering wheel angle and not angle of the wheels?

        # Update the last time.
        self.last_stamp = rospy.get_time()

        return throttle, brake, 0.
