#!/usr/bin/env python

import json
import rospy
from simulation_msg.srv import ObstacleServiceResponse, ObstacleService
from simulation_msg.msg import Slot, Radar, Coor


def get_obstacles(req):
    with open("/Components/common/config/slot.json", 'r') as f:
        slot = json.load(f)

    return ObstacleServiceResponse(
        slots=[Slot(corner=[Coor(x=x["x"], y=x["y"]) for x in s])
               for s in slot["slots"]],
        radar_points=[Radar(radar=[Coor(x=x["x"], y=x["y"]) for x in r]) for r in slot["radar_points"]])


def Server():
    rospy.init_node("radars")
    s = rospy.Service("/get_obstacles", ObstacleService, get_obstacles)
    rospy.spin()


if __name__ == "__main__":
    Server()
