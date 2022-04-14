from sim.outputs.OutputSystem import OutputSystem
from sim.constants import ROBOT
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

class AnimationOutput(OutputSystem):
    def __init__(self):
        super().__init__()
        """TODO: create figure to animate as the simulation runs"""
        self.fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2)
        self.fig.subplots_adjust(hspace=2)
        self.fig.subplots_adjust(wspace=0.5)
        self.ax1 = ax1
        self.ax2 = ax2
        self.ax3 = ax3
        self.ax4 = ax4
        self.ax5 = ax5
        self.ax6 = ax6

    def process(self, state, inpt, outpt, timestamp):
        """save data to dynamic memory"""
        super().process(state, inpt, outpt, timestamp)

        def animate(i=None):
            df = pd.DataFrame.from_dict(self._data)

            timestamps = list(df["timestamp"])
            ml = list(df["motor_l"])
            mr = list(df["motor_r"])

            x = list(df["x"])
            y = list(df["y"])
            theta = list(df["theta"])
            theta_dot = list(df["theta_dot"])

            lf = list(df["lidar_f"])
            lr = list(df["lidar_r"])

            mag_x = list(df["mag_x"])
            mag_y = list(df["mag_y"])
            gyro_z = list(df["gyro_z"])

            if(i):
                timestamps = timestamps[:i]
                ml = ml[:i]
                mr = mr[:i]
                x = x[:i]
                y = y[:i]
                theta = theta[:i]
                theta_dot = theta_dot[:i]
                lf = lf[:i]
                lr = lr[:i]
                mag_x = mag_x[:i]
                mag_y = mag_y[:i]
                gyro_z = gyro_z[:i]

            ax1 = self.ax1
            ax2 = self.ax2
            ax3 = self.ax3
            ax4 = self.ax4
            ax5 = self.ax5
            ax6 = self.ax6

            ax1.clear()
            ax2.clear()
            ax3.clear()
            ax4.clear()
            ax5.clear()
            ax6.clear()

            ax1.plot(timestamps, ml, label="motor_l")
            ax1.plot(timestamps, mr, label="motor_r")
            ax1.legend()
            ax1.set_xlabel("Time (s)")
            ax1.set_ylabel("Input Value")
            ax1.set_title("Inputs")

            ax2.plot(timestamps, x, label="x")
            ax2.plot(timestamps, y, label="y")
            ax2.legend()
            ax2.set_xlabel("Time (s)")
            ax2.set_ylabel("Position (m)")
            ax2.set_title("X and Y position")

            ax3.plot(timestamps, theta, label="theta")
            ax3.legend()
            ax3.set_xlabel("Time (s)")
            ax3.set_ylabel("Angle (rad)")
            ax3.set_title("Angle")

            ax4.plot(timestamps, theta_dot, label="theta_dot")
            ax4.legend()
            ax4.set_xlabel("Time (s)")
            ax4.set_ylabel("Angular Velocity (rad/s)")
            ax4.set_title("Angular Velocity")

            ax5.plot(timestamps, lf, label="lidar_f")
            ax5.plot(timestamps, lr, label="lidar_r")
            ax5.legend()
            ax5.set_xlabel("Time (s)")
            ax5.set_ylabel("Distance (m)")
            ax5.set_title("Lidar Outputs")

            ax6.plot(timestamps, mag_x, label="mag_x")
            ax6.plot(timestamps, mag_y, label="mag_y")
            ax6.plot(timestamps, gyro_z, label="gyro_z")
            ax6.legend()
            ax6.set_xlabel("Time (s)")
            ax6.set_ylabel("Value")
            ax6.set_title("Gyro and Magnetic Field Outputs")

        animate()
        plt.draw()
        plt.pause(0.001)

    def make_output(self):
        """make proper output from the data
        Called once after the simulation run is done"""
        pass






