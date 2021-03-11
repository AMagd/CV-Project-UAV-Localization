#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
from realsense import RealSense
import cv2
import math
rs = RealSense()

def nothing(x):
    pass

def publish_pose():
    pub = rospy.Publisher('localization', geometry_msgs.msg.PoseStamped, queue_size=10)
    rospy.init_node('LED_DETECTION', anonymous=True)
    rate = rospy.Rate(30)  # 10hz

    try:
        Win = 'RealSense_LED_Detection'
        # rWin = 'RealSense_Red_Detection'
        cv2.namedWindow(Win, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("hBar", Win, 30, 255, nothing)
        while True:
            hBar = cv2.getTrackbarPos("hBar", Win)

            images, wait_flag, landing_point = rs.update(hBar)

            #### currently put to 0, because it is unknown
            # xq, yq, zq, wq = eul2quat(landing_point[3], landing_point[4], landing_point[5])
            xq, yq, zq, wq = eul2quat(0, 0, 0)

            pose_goal = geometry_msgs.msg.PoseStamped()
            pose_goal.pose.position.x = landing_point[0]
            pose_goal.pose.position.y = landing_point[1]
            pose_goal.pose.position.z = landing_point[2]
            pose_goal.pose.orientation.x = xq
            pose_goal.pose.orientation.y = yq
            pose_goal.pose.orientation.z = zq
            pose_goal.pose.orientation.w = wq
            
            rospy.loginfo(pose_goal)
            pub.publish(pose_goal)

            if wait_flag:
                # Show images
                cv2.namedWindow(Win, cv2.WINDOW_AUTOSIZE)
                cv2.imshow(Win, images[0])
                cv2.imshow("IR", images[1])
                cv2.setWindowTitle(Win, 'RealSense FPS:' + str(rs.fps.get()))
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break

            flag_first = 0
            rate.sleep()

    finally:
        # Stop streaming
        rs.pipeline.stop()

def eul2quat(roll,pitch,yaw):
    eul=[    roll,    pitch   , yaw]
    c = [math.cos(eul[0]/2),math.cos(eul[1]/2),math.cos(eul[2]/2)]
    s = [math.sin(eul[0]/2),math.sin(eul[1]/2),math.sin(eul[2]/2)]
    q = [c[ 1-1]*c[ 2-1]*c[ 3-1] - s[ 1-1]*s[ 2-1]*s[ 3-1], s[ 1-1]*c[ 2-1]*c[ 3-1] + c[ 1-1]*s[ 2-1]*s[ 3-1],-s[ 1-1]*c[ 2-1]*s[ 3-1] + c[ 1-1]*s[ 2-1]*c[ 3-1],c[ 1-1]*c[ 2-1]*s[ 3-1] + s[ 1-1]*s[ 2-1]*c[ 3-1]]
    return q
    


if __name__ == '__main__':
    publish_pose()
