#!/usr/bin/env python
#-- coding: utf-8 --

import rospy

from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image 
from fiducial_msgs.msg import Fiducial, FiducialArray
from ar_track_alvar_msgs.msg import AlvarMarkers


class Sign():
    def __init__(self):
        self.sign_id = 0
        # rospy.Subscriber("/fiducial_vertices", FiducialArray, self.child_sign_callback)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.child_sign_callback)
        self.sign_id_pub = rospy.Publisher("sign_id", Int32, queue_size=1)
        self.pub_cnt = 0

    def child_sign_callback(self, _data):
        # rospy.loginfo("_data: %s", _data)
        print(_data.markers[0].id)
        # if _data.markers:  # 마커가 있는 경우에만 실행
        #     for marker in _data.markers:
        #         print("Marker ID:", marker.id)
        #         print("Position:", marker.pose.pose.position)
        #         print("Orientation:", marker.pose.pose.orientation)
        # else:
        #     print("No markers detected")
        if (len(_data.markers) > 0 ) :
            self.sign_id = _data.markers[0].id
            rospy.loginfo("################## ID : {}".format(self.sign_id))
            self.sign_id_pub.publish(self.sign_id)
            print("@@@@@@@@@@@")
            self.pub_cnt = 0
        else :
            self.pub_cnt += 1
        if self.pub_cnt > 20:
            self.sign_id_pub.publish(0)
            self.pub_cnt = 0
def run():
    rospy.init_node("signid")
    newclass = Sign()
    rospy.spin()


if __name__ == '__main__':
    run()