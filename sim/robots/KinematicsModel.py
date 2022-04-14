from sim.robots.RobotSystem import *
import numpy as np

class KinematicsModel(RobotSystem):
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

    def _kinematics_model(self, delta_t):
        """TODO: Your kinematics equations here
        :param delta_t: timestep size"""
        # self.state = f(self.state, self.inpt)
        X_IND = 0
        Y_IND = 1
        THETA_IND = 2
        THETA_DOT_IND = 3
        SX = 4
        SY = 5

        omega_l, omega_r = self.inpt
        vel_l = omega_l * ROBOT.WHEEL_RADIUS
        vel_r = omega_r * ROBOT.WHEEL_RADIUS

        theta_dot = (vel_l - vel_r) / ROBOT.WIDTH
        rot = np.pi / 2 - self.state[THETA_IND]

        x, y = self.state[X_IND], self.state[Y_IND]
        pos = np.array([[self.state[X_IND]], [self.state[Y_IND]]])
        c, s = np.cos(rot), np.sin(rot)
        rot_mat = np.array([[c, -1*s], [s, c]])
        inv_mat = np.array([[c, s], [-1*s, c]])
        if(vel_l == vel_r):
            translation = np.array([[0], [vel_l*delta_t]])
        else:
            radius = (ROBOT.WIDTH / 2) * ((vel_l + vel_r) / (vel_r - vel_l))
            c, s = np.cos(np.abs(theta_dot * delta_t)), np.sin(np.abs(theta_dot * delta_t))
            translation = np.array([[radius - radius * c], [radius * s]])

        pos = inv_mat @ (rot_mat @ pos + translation)
        pos = [float(i) for i in pos.reshape((pos.shape[0],))]

        """
        corners = [(ROBOT.WIDTH/2, ROBOT.WHEEL_RADIUS),
                   (-1*ROBOT.WIDTH/2, ROBOT.WHEEL_RADIUS),
                   (ROBOT.WIDTH/2, ROBOT.WHEEL_RADIUS - ROBOT.LENGTH),
                   (-1*ROBOT.WIDTH/2, ROBOT.WHEEL_RADIUS - ROBOT.LENGTH)]
        corners = [np.array(i).reshape((2,1)) for i in corners]
        curr_corners = [(inv_mat @ i) + np.array([[x], [y]]) for i in corners]
        valid_move = True
        for c in curr_corners:
            cx, cy = c[0], c[1]
            if(np.abs(cx) > 0.5 or np.abs(cy) > 0.5):
                valid_move = False
                break
        
        # for DEBUG
        #valid_move = True
        
        valid_move = True
        if(not valid_move):
            self.state[THETA_DOT_IND] = 0.0
        else:
        """
        self.state[X_IND] = float(pos[X_IND])
        self.state[Y_IND] = float(pos[Y_IND])
        self.state[THETA_IND] = float((self.state[THETA_IND] + theta_dot*delta_t) % (2*np.pi))
        self.state[THETA_DOT_IND] = float(theta_dot)


        def rotation_mat(theta):
            c, s = np.cos(theta), np.sin(theta)
            rot = np.array([[c, -s], [s, c]])
            return rot

        theta = self.state[THETA_IND]
        angle = np.array([np.cos(theta), np.sin(theta)])
        pos = np.array([self.state[X_IND], self.state[Y_IND]])
        rot_mat = rotation_mat(-np.pi/2)
        sensor_state = pos + ROBOT.SENSOR_POS[0]*angle + ROBOT.SENSOR_POS[1]*(rot_mat@angle.reshape((2,1)).reshape((2,)))
        
        self.state[SX] = float(sensor_state[0])
        self.state[SY] = float(sensor_state[1])

    def _sensor_model(self):
        """TODO: Your sensor equations here"""
        # self.outpt = h(self.state, self.inpt)

        LID_F_IND = 0
        LID_R_IND = 1
        MAG_X_IND = 2
        MAG_Y_IND = 3
        GYRO_IND = 4

        X_IND = 0
        Y_IND = 1
        THETA_IND = 2
        THETA_DOT_IND = 3



        def get_lidar(x, y, theta):
            # DEBUG
            return 1.0

            endpts = []
            orient = np.array([np.cos(theta), np.sin(theta)])
            if(np.cos(theta) == 0):
                endpts = [np.array([0, 0.5*np.sin(theta)])]
            elif(np.sin(theta) == 0):
                endpts = [np.array([0.5*np.cos(theta), 0])]
            else:
                m = orient[1] / orient[0]
                edges = [-0.5, 0.5]
                for x_end in edges:
                    endpts.append(np.array([x_end, m * x_end - m * x + y]))
                for y_end in edges:
                    endpts.append(np.array([(y_end - y + m * x) / m, y_end]))

            pos = np.array([x, y])
            best_dist = None
            for endpt in endpts:
                diff = endpt - pos
                if(np.dot(diff, orient) < 0):
                    continue
                dist = np.sqrt(diff @ diff.T)
                if(not best_dist or dist < best_dist):
                    best_dist = dist
            return float(best_dist)

        # distances
        x = self.state[X_IND]
        y = self.state[Y_IND]
        theta = self.state[THETA_IND]
        self.outpt[LID_F_IND] = get_lidar(x, y, theta)
        self.outpt[LID_R_IND] = get_lidar(x, y, (theta-(np.pi/2)) % (2*np.pi))
        self.outpt[MAG_X_IND] = float(np.cos(x))
        self.outpt[MAG_Y_IND] = float(np.sin(y))
        self.outpt[GYRO_IND] = self.state[THETA_DOT_IND]



