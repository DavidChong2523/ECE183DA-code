from sim.robots.RobotSystem import *
import sim.utils as utils
import numpy as np

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
        
    def drive(self, inpt, timestamp):
        """drive the robot to the next state
        :param inpt: left, right wheel velocities
        :return full state feedback"""
        self.inpt = tuple(ROBOT.SERVO_SPEED * i for i in inpt)
        self._kinematics_model(timestamp-self._t_minus_1)
        self._t_minus_1 = timestamp
        return self.state

    def sense(self):
        """generate the sensor reading"""
        self._sensor_model()
        return self.outpt

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

        if(vel_l == vel_r):
            curr_state = np.array([x, y, theta])
            unit_vec = np.array([np.cos(theta), np.sin(theta), 0])
            update = (vel_l*delta_t) * unit_vec + curr_state
        else:
            omega = (vel_r - vel_l) / ROBOT.WIDTH
            R = (ROBOT.WIDTH / 2) * ((vel_l + vel_r) / (vel_r - vel_l))

            ICC = [x - R*np.sin(theta), y + R*np.cos(theta)]
            coef = np.array([
                [np.cos(omega*delta_t), -np.sin(omega*delta_t), 0],
                [np.sin(omega*delta_t), np.cos(omega*delta_t), 0],
                [0, 0, 1]
            ])
            update = coef @ np.array([x-ICC[0], y-ICC[1], theta]) + np.array([ICC[0], ICC[1], omega*delta_t])

        self.state[ROBOT.I_X] = update[0]
        self.state[ROBOT.I_Y] = update[1]
        self.state[ROBOT.I_THETA] = update[2]

        def rotation_mat(theta):
            c, s = np.cos(theta), np.sin(theta)
            rot = np.array([[c, -s], [s, c]])
            return rot

        angle = np.array([np.cos(theta), np.sin(theta)])
        pos = np.array([x, y])
        rot_mat = rotation_mat(-np.pi/2)
        sensor_state = pos + ROBOT.SENSOR_POS[0]*angle + ROBOT.SENSOR_POS[1]*(rot_mat@angle.reshape((2,1)).reshape((2,)))
        #print(self.state)
        self.state[3] = float(sensor_state[0])
        self.state[4] = float(sensor_state[1])
        
    def _sensor_model(self):
        """TODO: Your sensor equations here"""
        theta = self.state[ROBOT.I_THETA]
        sensor_array = self.get_sensor_positions()
        sensor_readings = []
        for spos in sensor_array:
            image_pos = utils.env2image(spos, self.image_start)
            image_len = ENVIRONMENT.DISPLAY_SCALE * ROBOT.SENSOR_LENGTH
            image_width = ENVIRONMENT.DISPLAY_SCALE * ROBOT.SENSOR_WIDTH
            
            reading = utils.sensor_reading(self.env, image_pos, theta, (image_len, image_width))
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



