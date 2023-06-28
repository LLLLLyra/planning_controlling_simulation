#!/usr/bin/env python

import sys
import signal
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rospy
import tf_conversions
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from simulation_msg.msg import CarControlMsg

from draw_tools import plotVehicle


def quit(signum, frame):
    if signum in [signal.SIGINT, signal.SIGTERM]:
        sys.exit()


class ShowTime:
    def __init__(self):
        rospy.init_node("show")
        self.rf_path = None
        self.tra_path = None
        self.real_path = None
        self.obstacles = None
        self.tp = None
        self.delta = None
        self.reference_path_sub = rospy.Subscriber(
            "/reference_path", Path, self.reference_path_callback)
        self.reference_path_sub = rospy.Subscriber(
            "/trajectory", Path, self.trajectory_callback)
        self.real_path_sub = rospy.Subscriber(
            "/real_path", Path, self.real_path_callback)
        self.target_points_sub = rospy.Subscriber(
            "/target_point", Marker, self.target_point_call_back)
        self.obstacles_sub = rospy.Subscriber(
            "/obstacles", Path, self.obstacles_call_back)
        self.cmd_sub = rospy.Subscriber(
            "/car_control", CarControlMsg, self.cmd_call_back)
        signal.signal(signal.SIGINT, quit)
        signal.signal(signal.SIGTERM, quit)
        self.fig, self.ax = plt.subplots(1, 1)
        self.ax.set_xlabel("x[m]")
        self.ax.set_ylabel("y[m]")
        self.ax.axis("equal")
        self.ax.legend()
        ani = animation.FuncAnimation(
            fig=self.fig, func=self.update, interval=50, blit=False)
        plt.show()

    def trajectory_callback(self, rf):
        self.tra_path = [(p.pose.position.x, p.pose.position.y)
                         for p in rf.poses]

    def reference_path_callback(self, rf):
        self.rf_path = [(p.pose.position.x, p.pose.position.y)
                        for p in rf.poses]
        if not self.rf_path:
            return
        self.minx, self.miny = np.min(np.array(self.rf_path), axis=0)
        self.maxx, self.maxy = np.max(np.array(self.rf_path), axis=0)

    def real_path_callback(self, real):
        self.real_path = [
            (p.pose.position.x, p.pose.position.y,
             tf_conversions.transformations.euler_from_quaternion([
                 p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w
             ])[-1] + np.pi / 2)
            for p in real.poses]

    def target_point_call_back(self, tp):
        self.tp = tp.pose.position

    def obstacles_call_back(self, obstacles):
        self.obstacles = [(p.pose.position.x, p.pose.position.y)
                          for p in obstacles.poses]

    def cmd_call_back(self, cmd):
        self.delta = np.deg2rad(cmd.steering_angle) / 17.2

    def update(self, *args):
        self.ax.clear()
        if self.tra_path:
            traj_x, traj_y = zip(*self.tra_path)
            self.ax.plot(traj_x, traj_y, "-r", linewidth=2, label="planned")
        if self.rf_path:
            ref_x, ref_y = zip(*self.rf_path)
            self.ax.plot(ref_x, ref_y, "-c", linewidth=1,
                         label="reference line")
        if self.obstacles:
            ob_x, ob_y = zip(*self.obstacles)
            self.ax.scatter(ob_x, ob_y, c='b', s=5, label="obstacles")
        if self.real_path and self.tp and self.delta is not None:
            traj_ego_x, traj_ego_y, theta = zip(*self.real_path)
            self.ax.plot(traj_ego_x, traj_ego_y, "-b",
                         linewidth=2, label="trajectory")
            self.ax.plot(self.tp.x, self.tp.y, "og",
                         ms=5, label="target point")
            plotVehicle(traj_ego_x[-1], traj_ego_y[-1],
                        theta[-1], self.ax, self.delta)
            # self.ax.set_xlim(self.minx - (self.maxx - self.minx)
            #                  * 0.2, self.maxx + (self.maxx - self.minx) * 0.2)
            # self.ax.set_ylim(self.miny - (self.maxy - self.miny)
            #                  * 0.2, self.maxy + (self.maxy - self.miny) * 0.2)
        self.ax.legend()
        return self.ax


if __name__ == "__main__":
    show = ShowTime()
