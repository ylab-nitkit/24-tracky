#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from std_msgs.msg import String
import whisper
import numpy as np
import threading
import sounddevice as sd

# グローバル変数
is_tracking = True
model = YOLO('yolov8n.pt')
bridge = CvBridge()
pub = rospy.Publisher('chatter', String, queue_size=10)
cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # ドローン制御用

# Whisperモデルのロード
whisper_model = whisper.load_model("base")
# 音声データをnumpy配列に変換する関数
def audio_to_numpy(audio_data):
    audio_np = np.array(audio_data, dtype=np.float32)
    return audio_np / np.max(np.abs(audio_np))  # normalize to [-1, 1]

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
                
        #confの制限は随時調整
        if conf > 0.5 and is_tracking and x_center is not None and y_center is not None:
            # ドローンの制御コマンドを生成
            twist = Twist()
            # X 軸方向の制御
            twist.linear.x = 0.5 * (y_center - orig.shape[0] / 2) / (orig.shape[0] / 2)  # 上下移動
            # Y 軸方向の制御
            twist.linear.y = 0.5 * (x_center - orig.shape[1] / 2) / (orig.shape[1] / 2)  # 左右移動
            # Z 軸方向の制御（高度制御、必要に応じて調整）
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = (x_center - orig.shape[1] / 2) / (orig.shape[1] / 2)  # x 軸方向の制御
            # ドローンの動きに合わせてコマンドを調整
            cmd_pub.publish(twist)

            talk_str = String()
            talk_str.data = "Person detected and tracking"
            pub.publish(talk_str.data)
        
        cv2.imshow("YOLOv8 Inference", annotated_frame)
        cv2.waitKey(100)
    except Exception as err:
        rospy.logerr(f"Error processing image: {err}")

def audio_recognition():
    # マイクからの音声入力の設定
    sample_rate = 16000
    # マイクからの音声入力の設定
    def callback(indata, frames, time, status):
        global is_tracking
        if status:
            rospy.logerr(status)
        audio_np = audio_to_numpy(indata)
        result = whisper_model.transcribe(audio_np, sampling_rate=sample_rate)
        transcript = result['text'].strip().lower()
        
        if '停止' in transcript:
            is_tracking = False
            rospy.loginfo("トラッキングを停止します")
            if '開始' in transcript:
                is_tracking = True
                rospy.loginfo("トラッキングを開始します")
    # ストリームの設定
    with sd.InputStream(callback=callback, channels=1, samplerate=sample_rate, dtype='int16'):
        rospy.loginfo("音声認識スレッドが開始されました")
        rospy.spin()

def start_node():
    rospy.init_node('img_proc')
    rospy.loginfo('img_proc node started')
    rospy.Subscriber("/ardrone/front/image_raw", Image, process_image)

    # 音声認識スレッドの開始
    audio_thread = threading.Thread(target=audio_recognition)
    audio_thread.start()
    rospy.loginfo("Sound")

    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
