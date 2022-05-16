from sim.robots.RobotSystem import *
import sim.utils as utils
import numpy as np
import collections

class KinematicsModel(RobotSystem):
    def __init__(self, environment):
        """init with a specific initial state (optional) """
        super().__init__()
        self.state = ENVIRONMENT.INITIAL_ROBOT_STATE
        self.inpt = [0.0 for k in INPUT_SPACE]
        self.outpt = [[0.0] for k in OUTPUT_SPACE]

        # numpy array [x, y] image coords
        self.image_start = np.array([100, 100])
        self.env = environment

        # pid control
        self.last_reading_time = 0
        self.n = 1
        self.pid_history = 0
        self.prev_reading = 0
        self.pid_readings = collections.deque(maxlen=1)

    def drive(self, inpt, timestamp):
        """drive the robot to the next state
        :param inpt: left, right wheel velocities
        :return full state feedback"""
        self.inpt = tuple(ROBOT.SERVO_SPEED * i for i in inpt)
        
        # pid control
        self.inpt = self.pid_control()

        self._kinematics_model(timestamp-self._t_minus_1)
        self._t_minus_1 = timestamp
        return self.state

    def sense(self):
        """generate the sensor reading"""
        self._sensor_model()
        return self.outpt

    """
    convert forward velocity and angular velocity to wheel angular velocities for differential drive
    linear velocity: https://stackoverflow.com/questions/29226752/differential-drive-robots-converting-wheel-speeds-to-lin-ang-velocities
    """
    def velocity_to_input(self, vel, omega):
        # see kinematics function for conversion derivation
        wheel_vel_sum = 2*vel / ROBOT.WHEEL_RADIUS # vel_r + vel_l
        wheel_vel_diff = omega * ROBOT.WHEEL_SPACING # vel_r - vel_l

        vel_r = 0.5 * (wheel_vel_sum + wheel_vel_diff)
        vel_l = wheel_vel_sum - vel_r
        
        # not currently converting to input
        return (vel_r, vel_l)


    def pid_control(self):
        readings = self.outpt[ROBOT.I_SENSE]
        if(not any(readings)):
            self.last_reading_time += 1
        else:
            self.last_reading_time = 0
        
        LINEAR_VEL = 12 # in/s

        def sensor_to_vel(reading_pos):
            # quadratic scale
            omega = 1*(reading_pos**2) + 0*reading_pos
            return omega

        values = [sensor_to_vel(i-ROBOT.NUM_SENSORS//2)*r for i, r in enumerate(readings)]
        curr_p = sum(values) / len(values)

        self.pid_readings.append(curr_p)
        # moving average
        p = sum(self.pid_readings) / len(self.pid_readings)
        
        self.pid_history += p
        i = self.pid_history

        d = self.prev_reading - p# - self.prev_reading
        self.prev_reading = p

        # pid is angular velocity
        pid = 0.2*p +0.00*i + 0*d
        
        # sensor offset
        offset_vel = LINEAR_VEL# - ROBOT.WHEEL_SPACING*pid

        inpt = self.velocity_to_input(offset_vel, pid)

        print("PID:", pid, "INPT:", inpt)#, "VALUES:", values)

        # Manual control
        if((sum(self.pid_readings) == 0 and self.last_reading_time > 1) or False):
            # direct velocity
            self.inpt = (self.inpt[0], self.inpt[1])
            #return self.velocity_to_input(20, np.pi/12)
            return self.inpt

        return inpt

    """
    https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
    """
    def _kinematics_model(self, delta_t):
        """TODO: Your kinematics equations here
        :param delta_t: timestep size"""

        omega_l, omega_r = self.inpt
        vel_l = omega_l * ROBOT.WHEEL_RADIUS
        vel_r = omega_r * ROBOT.WHEEL_RADIUS

        x = self.state[ROBOT.I_X]
        y = self.state[ROBOT.I_Y]
        theta = self.state[ROBOT.I_THETA]

        # convert from center of robot to center of axle
        axle_pos = np.array([x, y]) + ROBOT.AXLE_POS * np.array([np.cos(theta), np.sin(theta)])
        x, y = axle_pos[0], axle_pos[1]
        
        if(vel_l == vel_r):
            curr_state = np.array([x, y, theta])
            unit_vec = np.array([np.cos(theta), np.sin(theta), 0])
            update = (vel_l*delta_t) * unit_vec + curr_state
        else:
            omega = (vel_r - vel_l) / ROBOT.WHEEL_SPACING
            R = (ROBOT.WHEEL_SPACING / 2) * ((vel_l + vel_r) / (vel_r - vel_l))

            ICC = [x - R*np.sin(theta), y + R*np.cos(theta)]
            coef = np.array([
                [np.cos(omega*delta_t), -np.sin(omega*delta_t), 0],
                [np.sin(omega*delta_t), np.cos(omega*delta_t), 0],
                [0, 0, 1]
            ])
            update = coef @ np.array([x-ICC[0], y-ICC[1], theta]) + np.array([ICC[0], ICC[1], omega*delta_t])

        # convert from center of axle to center of robot
        center_pos = update[:2] + ROBOT.AXLE_POS * -np.array([np.cos(update[2]), np.sin(update[2])])
        update[:2] = center_pos

        self.state[ROBOT.I_X] = update[0]
        self.state[ROBOT.I_Y] = update[1]
        self.state[ROBOT.I_THETA] = update[2]

        # simulate noise
        self.actuation_noise()

    def actuation_noise(self):
        trans_prob = ROBOT.TRANS_NOISE
        rot_prob = ROBOT.ROT_NOISE
        trans_mag = ROBOT.TRANS_MAG
        rot_mag = ROBOT.ROT_MAG
        
        if(np.random.uniform() < trans_prob):
            x_noise = np.random.uniform(low=-trans_mag, high=trans_mag)
            y_noise = np.random.uniform(low=-trans_mag, high=trans_mag)
            self.state[ROBOT.I_X] += x_noise
            self.state[ROBOT.I_Y] += y_noise
        if(np.random.uniform() < rot_prob):
            rot_noise = np.random.uniform(low=-rot_mag, high=rot_mag)
            self.state[ROBOT.I_THETA] = (self.state[ROBOT.I_THETA] + rot_noise) % (2*np.pi)

        
    def _sensor_model(self):
        """TODO: Your sensor equations here"""
        theta = self.state[ROBOT.I_THETA]
        sensor_array = self.get_sensor_positions()
        sensor_readings = []
        for spos in sensor_array:
            image_pos = utils.env2image(spos, self.image_start)
            image_len = ENVIRONMENT.DISPLAY_SCALE * ROBOT.SENSOR_LENGTH
            image_width = ENVIRONMENT.DISPLAY_SCALE * ROBOT.SENSOR_WIDTH
            
            reading = utils.sensor_reading(self.env, image_pos, theta, (image_len, image_width), noise=ROBOT.S_NOISE)
            sensor_readings.append(reading)

        self.outpt[ROBOT.I_SENSE] = sensor_readings

    def get_sensor_positions(self):
        x = self.state[ROBOT.I_X]
        y = self.state[ROBOT.I_Y]
        theta = self.state[ROBOT.I_THETA]

        sensor_pos = np.array([x, y]) + utils.rotation_mat(theta) @ ROBOT.SENSOR_POS
        
        sep = ROBOT.SENSOR_WIDTH + ROBOT.SENSOR_SEP
        unit_vec = utils.rotation_mat(-np.pi/2) @ np.array([np.cos(theta), np.sin(theta)])
        sensor_array = [sensor_pos + (i-ROBOT.NUM_SENSORS//2)*sep*unit_vec for i in range(ROBOT.NUM_SENSORS)]
        return sensor_array



