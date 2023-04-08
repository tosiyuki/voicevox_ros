#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from voicevox_ros.msg import TtsMessage
from std_msgs.msg import Int32

if __name__ == '__main__':
    rospy.init_node('voicevox_sample')
    voicevox_publisher = rospy.Publisher('/tts/voicevox', TtsMessage, queue_size=10)
    change_speaker_publisher = rospy.Publisher('/tts/voicevox/change_speaker', Int32, queue_size=10)

    tts_message = TtsMessage()
    #tts_message.speed_scale = 1.0

    while not rospy.is_shutdown():
        tts_message.text = input()
        voicevox_publisher.publish(tts_message)