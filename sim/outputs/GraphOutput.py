from sim.outputs.OutputSystem import OutputSystem
from sim.constants import ROBOT
import matplotlib.pyplot as plt
import pandas as pd

class GraphOutput(OutputSystem):
    def __init__(self, figname=None):
        super().__init__()
        self.figname = figname

    def make_output(self):
        """TODO: Add your figure generation code here"""
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

        fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2)
        fig.subplots_adjust(hspace=2)
        fig.subplots_adjust(wspace=0.5)

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


        plt.show()

        if(self.figname):
            fig.savefig(self.figname)


