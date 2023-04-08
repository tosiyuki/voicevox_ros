#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import wave

import requests
import rospy
import pygame

from voicevox_ros.msg import TtsMessage

class VoicevoxTextToSpeechRos:
    def __init__(self, speaker, host, port, sound_file_path):
        self.speaker = speaker
        self.audio_query_url = f'http://{host}:{port}/audio_query'
        self.sybthesis_url = f'http://{host}:{port}/synthesis'
        self.sound_file_path = sound_file_path

        pygame.mixer.init()
        self.sub_text_to_speech = rospy.Subscriber('/tts/voicevox', TtsMessage, self.callback_text_to_speech)

    def callback_text_to_speech(self, data: TtsMessage):
        rospy.loginfo(f'call callback_text_to_speech: {data}')
        self._create_sound(data.text, data.speed_scale)
        self._play_sound()

    def _create_sound(self, text, speed_scale):
        params = (
            ('text', text),
            ('speaker', self.speaker),
        )
        response1 = requests.post(
            self.audio_query_url,
            params=params
        )

        response1 = response1.json()
        if speed_scale:
            response1['speedScale'] = speed_scale

        headers = {'Content-Type': 'application/json',}
        response2 = requests.post(
            self.sybthesis_url,
            headers=headers,
            params=params,
            data=json.dumps(response1)
        )

        with wave.open(self.sound_file_path, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(24000)
            wf.writeframes(response2.content)

    def _play_sound(self):
        pygame.mixer.music.load(self.sound_file_path)
        pygame.mixer.music.play()

    def end(self):
        pygame.quit()


if __name__ == "__main__":
    rospy.init_node('voicevox_ros', anonymous=True)
    speaker = rospy.get_param('voicevox_text_to_speech', 'speaker')['speaker']
    host = rospy.get_param('voicevox_text_to_speech', 'host')['host']
    port = rospy.get_param('voicevox_text_to_speech', 'port')['port']
    sound_file_path = rospy.get_param('voicevox_text_to_speech', 'sound_file_path')['sound_file_path']

    rospy.loginfo(f'speaker:{speaker}\nhost:{host}\nport:{port}\nsound_file_path:{sound_file_path}')

    voicevox_tts = VoicevoxTextToSpeechRos(speaker, host, port, sound_file_path)
    try:
        rospy.spin()
    finally:
        voicevox_tts.end()
        exit()

