#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import whisper
import pyaudio
import wave
import numpy as np
import time
import rospy
from std_msgs.msg import String

rospy.init_node('whisper_test', anonymous=False)
pub = rospy.Publisher('whisper_result', String, queue_size=10)
rate = rospy.Rate(100)

# Whisper モデルをロード
model = whisper.load_model("base")

# PyAudio 設定
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
RECORD_SECONDS = 5

# 音声認識関数
def recognize_speech():
    p = pyaudio.PyAudio()

    # マイクから音声を録音
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    print("音声を録音中...")
    frames = []

    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)

    print("録音完了。音声認識中...")

    stream.stop_stream()
    stream.close()
    p.terminate()

    # 録音した音声データを処理
    wf = wave.open("output.wav", 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

    # Whisperで音声認識
    result = model.transcribe("output.wav", language="ja")

    # 認識結果を返す
    return result['text']

# メインループ
while not rospy.is_shutdown():
    text = recognize_speech().strip()

    print(f"認識されたテキスト: {text}")

    msg = String()
    msg.data = text
    pub.publish(msg)
    rate.sleep()

    # 少し待ってから再度録音
    time.sleep(2)

