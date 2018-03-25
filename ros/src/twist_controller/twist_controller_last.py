import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
TEMP_STEER = -8


class Controller(object):
    def __init__(self, *args, **kwargs):

        self.accel_limit = kwargs['accel_limit']
        self.throttle_PID = PID(30, 3, 1) # TODO: optimize these weights on P, I, and D.
        self.last_stamp = None # ros timestamp of the last time control() was called.
        self.yaw_controller = YawController(kwargs['wheel_base'],
                                            kwargs['steer_ratio'],
                                            ONE_MPH,
                                            kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'])
        self.filter = LowPassFilter(0.2, 0.1)

        self.vehicle_mass = kwargs['vehicle_mass'] + kwargs['fuel_capacity'] * GAS_DENSITY
        self.wheel_radius = kwargs['wheel_radius']
        self.decel_limit = kwargs['decel_limit']
        self.brake_deadband = kwargs['brake_deadband']

        self.max_accel_torque = self.vehicle_mass * self.accel_limit * self.wheel_radius

    def control(self, target_x_vel, target_ang_vel, curr_x_vel, dbw_enabled):
        # TODO: Remove these hardcoded variables and use the parameters.
        #target_x_vel = SPEED_LIMIT_MPS
        #target_ang_vel = TEMP_STEER

        if self.last_stamp is None or not dbw_enabled:
            self.last_stamp = rospy.get_time() # On the first pass, just initialize last_stamp.
            # Reset the PID Controllers just in case.
            self.throttle_PID.reset()
            return 0.0, 0.0, 0.0

        # Note: It might be better to use the timestamps in the messages for better accuracy?
        curr_time = rospy.get_time()
        time_delta = curr_time - self.last_stamp
        self.last_stamp = curr_time

        # Note: vel_err is positive when car is too slow and negative when going too fast.
        vel_err = target_x_vel - curr_x_vel
        T = self.throttle_PID.step(vel_err, time_delta)

        # adjust throttle at low velocity
        if target_x_vel < 3 and curr_x_vel < 4:
          T = vel_err / time_delta
          T = min(self.accel_limit, T) if T >= 0. else max(self.decel_limit, T)

          if abs(T) < self.brake_deadband:
            T = 0.

          T = self.vehicle_mass * T * self.wheel_radius

        # adjust throttle and brake when speed is zero
        if target_x_vel < 1 and curr_x_vel < 1:
            throttle = 0.
            brake = 1.
        else:
            # compute throttle and brake
            throttle = T / self.max_accel_torque if T > 0. else 0
            brake = abs(T) if T <= 0. else 0.

        steer = self.yaw_controller.get_steering(target_x_vel,
                                                 target_ang_vel,
                                                 curr_x_vel)

        # TODO: Do we need to use the lowpass filter, why?
        # steer = self.filter.filt(steer)

        # # Update the last time.
        # self.last_stamp = rospy.get_time()

        return throttle, brake, steer
