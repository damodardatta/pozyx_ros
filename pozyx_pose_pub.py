#!/usr/bin/env python3
import pypozyx
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion

remote_id = 0x6839 #Slave Tag ID


def pozyx_pose_pub():
    pub = rospy.Publisher('pozyx_pose', Pose, queue_size=40)
    rospy.init_node('pozyx_pose')
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[0].device)
    except:
        rospy.loginfo("Pozyx not connected")
        return
    while not rospy.is_shutdown():
        coords = pypozyx.Coordinates()
        quat = pypozyx.Quaternion()
        pozyx.doPositioning(coords, pypozyx.POZYX_3D, remote_id=remote_id)
        pozyx.getQuaternion(quat, remote_id=remote_id)
        # rospy.loginfo("POS: %s, QUAT: %s" % (str(coords), str(quat)))
        pub.publish(Point(coords.x, coords.y, coords.z),
                    Quaternion(quat.x, quat.y, quat.z, quat.w))


if __name__ == '__main__':
    try:
        pozyx_pose_pub()
    except rospy.ROSInterruptException:
        pass