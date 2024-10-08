#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from std_msgs.msg import String
from std_msgs.msg import Empty
# Load a model
model = YOLO('yolov8n.pt')
# Predict with the model

bridge = CvBridge()
i = 0
pub =rospy.Publisher('chatter', String, queue_size=10)

def process_image(msg):
    try:
        conf = 0
        # global i
        # i=i+1
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        #img = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('image', img)
        cv2.imshow('image',orig)
        #mojiretu = "image" + str(i)+".png"
        #result = cv2.imwrite(mojiretu,orig)
        # cv2.waitKey(100)
        # if i>20:
        #     i = 0
        results = model(orig,save=True, classes=[0])
        annotated_frame = results[0].plot()
        #print(results)
        for res in results:
            clscount = [0] * len(res.names) 
            for box in res.boxes:
                conf = box.conf[0].item()   # 信頼度
                cls = int(box.cls[0].item()) # クラス
                clscount[cls] += 1   # クラスごとのカウント 
                clsnm = res.names[cls]   # クラス名
                pos = box.xywh[0]   # 座標
                #print("-----")
                #print(f"class = {cls}:{clsnm}") # クラス表示
                #print(f"conf = {conf:.5f}") # 信頼度表示
                #print(f"position = X:{pos[0].item()} Y:{pos[1].item()} width:{pos[2].item()} height:{pos[3].item()}")   # 座標表示
                #score = results.boxes.conf.cpu().numpy()[0]
        #print(score)
        if conf >0.5:
            talk_str = String()
            talk_str.data = "Person detected"
            pub.publish(talk_str.data)
        cv2.imshow("YOLOv8 Inference", annotated_frame)
        cv2.waitKey(100)
    except Exception as err:
        print(err)

def start_node():
    rospy.init_node('img_proc')
    rospy.loginfo('img_proc node started')
    rospy.Subscriber("/ardrone/front/image_raw", Image, process_image)
    # rospy.Publisher("/ardrone/takeoff",Empty)

    # cv2.waitKey(80000)

    # rospy.Publisher("/ardrone/land",Empty)
                    
# Sending Commands to AR-Drone

# The drone will takeoff, land or emergency stop/reset if a ROS std_msgs/Empty message is published to ardrone/takeoff, ardrone/land and ardrone/reset topics respectively.

# In order to fly the drone after takeoff, you can publish a message of type geometry_msgs::Twist to the cmd_vel topic:

# -linear.x: move backward
# +linear.x: move forward
# -linear.y: move right
# +linear.y: move left
# -linear.z: move down
# +linear.z: move up

# -angular.z: turn right
# +angular.z: turn left

# The range for each component should be between -1.0 and 1.0. The maximum range can be configured using ROS Parameters_ discussed later in this document.
# Hover Modes

# geometry_msgs::Twist has two other member variables angular.x and angular.y which can be used to enable/disable “auto-hover” mode. “auto-hover” is enabled when all six components are set to zero. If you want the drone not to enter “auto hover” mode in cases you set the first four components to zero, set angular.x and angular.y to arbitrary non-zero values.
# ,Empty,)

    #rospy.Subscriber("chatter", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass