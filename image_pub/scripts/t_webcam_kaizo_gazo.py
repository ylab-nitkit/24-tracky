#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from std_msgs.msg import String
import numpy as np
import l_webcam

# グローバル変数
is_tracking = False
model = YOLO('yolov8n.pt')
bridge = CvBridge()
pub = rospy.Publisher('chatter', String, queue_size=10)
cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # ドローン制御用

def process_image(msg):
    global is_tracking
    try:
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('image', orig)
        results = model(orig, save=True, classes=[0])
        annotated_frame = results[0].plot()
        
        conf = 0
        x_center, y_center = None, None
        for res in results:
            for box in res.boxes:
                conf = box.conf[0].item()   # 信頼度
                cls = int(box.cls[0].item()) # クラス
                clsnm = res.names[cls]   # クラス名
                pos = box.xyxy[0]   # 座標
                
                # バウンディングボックスの中心を計算
                x_center = (pos[0] + pos[2]) / 2
                y_center = (pos[1] + pos[3]) / 2
                is_tracking = True
                
        #confの制限は随時調整
        if conf > 0.5 and is_tracking == True and x_center is not None and y_center is not None:
            # ドローンの制御コマンドを生成
            twist = Twist()
            # X 軸方向の制御
            twist.linear.x = 0.01 
            #twist.linear.x = 0.0
            #twist.linear.y = 0.0
            # Y 軸方向の制御orig.shape[0]
            twist.linear.y = 0.01 
            # Z 軸方向の制御（高度制御、必要に応じて調整）
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = -0.01 * (x_center - orig.shape[1] / 2) / (orig.shape[1] / 2)  # x 軸方向の制御
            # ドローンの動きに合わせてコマンドを調整
            cmd_pub.publish(twist)

            talk_str = String()
            talk_str.data = "Person detected and tracking"
            pub.publish(talk_str.data)
        else:
            twist = Twist()
            twist.angular.z = 0.0
            print("False")
        cv2.imshow("YOLOv8 Inference", annotated_frame)
        cv2.waitKey(7)
    except Exception as err:
        rospy.logerr(f"Error processing image: {err}")

def start_node():
    rospy.init_node('object_tracking_node')
    rospy.loginfo('object_tracking_node started')
    rospy.Subscriber("/ardrone/front/image_raw", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
