#!/usr/bin/env python3
import signal
import sys
import matplotlib.pyplot as plt
import rospy
from matplotlib.animation import FuncAnimation
from visualization_msgs.msg import Marker
from matplotlib.ticker import MultipleLocator


# Helper function which is called when pressing Ctrl-C in the terminal or
# when the Matplotlib window is closed.
def shutdown():
    rospy.signal_shutdown('exit')
    sys.exit()


class Visualiser:
    def __init__(self, axis_limit):
        self.fig = plt.figure()
        self.ax = plt.axes()
        # Axis ticks and grid and grid spacing
        self.ax.xaxis.set_major_locator(MultipleLocator(1))
        self.ax.yaxis.set_major_locator(MultipleLocator(1))
        self.ax.set_aspect('equal')
        # Dashed linestyle
        plt.grid(linestyle="--")
        # r = red, o = circle markers.
        # See https://matplotlib.org/3.3.3/api/_as_gen/matplotlib.pyplot.plot.html
        self.plot, = plt.plot([], [], 'ro', markersize=3)
        self.x_data, self.y_data = [], []
        self.axis_limit = axis_limit

        # Shutdown the ROS node when we close the matplotlib window.
        self.fig.canvas.mpl_connect('close_event', lambda _: shutdown())

    def plot_init(self):
        # Set the plot extents ie. axis limits.
        self.ax.set_xlim(-self.axis_limit, self.axis_limit)
        self.ax.set_ylim(-self.axis_limit, self.axis_limit)
        return self.plot

    def callback(self, msg):
        # Replace the data to be plotted with the one from the message that we just received.
        self.x_data = [p.x for p in msg.points]
        self.y_data = [p.y for p in msg.points]

    def update_plot(self, frame):
        self.plot.set_data(self.x_data, self.y_data)
        return self.plot


if __name__ == "__main__":
    rospy.init_node('visualizer', disable_signals=True)
    # Ensure shutdown on ctrl-C
    signal.signal(signal.SIGINT, lambda _, __: shutdown())

    visualizer = Visualiser(axis_limit=25)
    subscriber = rospy.Subscriber('point_positions', Marker, visualizer.callback)

    # redraw interval 40 ms == 25 Hz
    animation = FuncAnimation(visualizer.fig, visualizer.update_plot, init_func=visualizer.plot_init, interval=40)

    plt.show(block=True)
