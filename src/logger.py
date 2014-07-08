#!/usr/bin/python

#   Calder Phillips-Grafflin

import rospy
from std_msgs.msg import *
from lightweight_vicon_bridge.msg import *

class Logger:

    def __init__(self, marker_topic, object_topic):
        rospy.on_shutdown(self.on_shutdown)
        self.markers = []
        self.objects = []

        if not (marker_topic or object_topic):
            print "At least one topic needed to subscribe to"
            exit(0)
        if marker_topic:
            self.marker_sub = rospy.Subscriber(marker_topic, MocapMarkerArray, self.marker_cb)
        if object_topic:
            self.object_sub = rospy.Subscriber(object_topic, MocapState, self.object_cb)

        sleep_rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            sleep_rate.sleep()

    def marker_cb(self, msg):
        self.markers.append(msg)

    def object_cb(self, msg):
        self.objects.append(msg)


    def on_shutdown(self):
        if self.markers:
            print "Logging stored marker values to disk..."
            f = open("markers.csv", "w")
            for frame in self.markers:
                line_str = ""
                # line_str += str(pos.tracker_name) + ','
                line_str += str(frame.header.stamp.secs) + ','
                line_str += str(frame.header.stamp.nsecs) + ','
                line_str += str(len(frame.markers)) + ','
                for m in frame.markers:
                    line_str += str(m.index) + ","
                    line_str += str(m.position.x) + ","
                    line_str += str(m.position.y) + ","
                    line_str += str(m.position.z) + ","
                line_str = line_str.rstrip(',')
                line_str += '\n'
                f.write(line_str)
            f.close()
            print "...done, exiting"

        if self.objects:
            print "Logging stored object values to disk..."
            f = open("objects.csv", "w")
            for frame in self.objects:
                line_str = ""
                # line_str += str(pos.tracker_name) + ','
                line_str += str(frame.header.stamp.secs) + ','
                line_str += str(frame.header.stamp.nsecs) + ','
                line_str += str(len(frame.tracked_objects)) + ','
                for o in frame.tracked_objects:
                    segment = o.segments[0]         #  All objects we use are 1 segment.

                    line_str += str(segment.name) + ","
                    line_str += str(int(segment.occluded)) + ","
                    line_str += str(segment.transform.translation.x) + ","
                    line_str += str(segment.transform.translation.y) + ","
                    line_str += str(segment.transform.translation.z) + ","
                    line_str += str(segment.transform.rotation.x) + ","
                    line_str += str(segment.transform.rotation.y) + ","
                    line_str += str(segment.transform.rotation.z) + ","
                    line_str += str(segment.transform.rotation.w) + ","
                line_str = line_str.rstrip(',')
                line_str += '\n'
                f.write(line_str)
            f.close()
            print "...done, exiting"

if __name__ == '__main__':
    rospy.init_node('logger')
    marker_topic = rospy.get_param("~markers_topic", "mocap_markers")
    object_topic = rospy.get_param("~objects_topic", "mocap_tracking" )
    Logger(marker_topic, object_topic)


