#!/usr/bin/python

#   Calder Phillips-Grafflin

import rospy
from std_msgs.msg import *
from lightweight_vicon_bridge.msg import *

class Logger:

    def __init__(self, marker_topic):
        rospy.on_shutdown(self.on_shutdown)
        self.positions = []
        self.position_sub = rospy.Subscriber(marker_topic, MocapMarkerArray, self.position_cb)
        sleep_rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            sleep_rate.sleep()

    def position_cb(self, msg):
        # cur_time = rospy.get_time()
        self.positions.append(msg)

    def on_shutdown(self):
        print "Logging stored values to disk..."
        f = open("positions.csv", "w")
        for pos in self.positions:
            line_str = ""
            # line_str += str(pos.tracker_name) + ','
            line_str += str(pos.header.stamp.secs) + ','
            line_str += str(pos.header.stamp.nsecs) + ','
            line_str += str(len(pos.markers)) + ','
            for p in pos.markers:
                line_str += str(p.index) + ","
                line_str += str(p.position.x) + ","
                line_str += str(p.position.y) + ","
                line_str += str(p.position.z) + ","
            line_str = line_str.rstrip(',')
            line_str += '\n'
            f.write(line_str)
        f.close()
        print "...done, exiting"

if __name__ == '__main__':
    rospy.init_node('logger')
    marker_topic = rospy.get_param("~markers_topic", "mocap_markers")
    Logger(marker_topic)
