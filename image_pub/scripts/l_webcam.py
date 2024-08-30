#!/usr/bin/env python
## coding: UTF-8
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
#import subprocess
import t_webcam_kaizo_gazo

def callback(message):
    rospy.loginfo("[%s]", message.data) # ターミナルへの表
    if "ストップ" in sub_whis:
        try:
            #subprocess.run(["rostopic", "pub", "/ardrone/land", "std_msgs/Empty"], check=True)
            t_webcam_kaizo_gazo.is_tracking = False 
            print("ドローンを停止します")
            if "スタート" in sub_whis:
                #subprocess.run(["rostopic", "pub", "/ardrone/takeoff", "std_msgs/Empty"], check=True)
                t_webcam_kaizo_gazo.is_tracking = True
                print("追従を開始します")
        except Exception as e:
            rospy.logerr(f"Failed to execute command: {e}")

rospy.init_node('listener')
sub = rospy.Subscriber('chatter', String, callback) # chatterというTopicを受信！受信したら上で定義したcallback関数を呼ぶ
#cmd_sub = rospy.Subscriber('/cmd_vel', Twist, callback)
sub_whis = rospy.Subscriber('whisper_result', String, callback)
    
rospy.spin()