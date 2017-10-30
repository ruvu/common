# MIT License
#
# Copyright (c) 2017 RUVU Robotics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from qi_session import QiSession
import rospy


class Proxy:

    def __init__(self):
        self._session = QiSession()

        rospy.loginfo("Connected")

        self.autonomous_life = self._session.get_service("ALAutonomousLife")
        self.motion = self._session.get_service("ALMotion")
        self.posture = self._session.get_service("ALRobotPosture")
        self.basic_awareness = self._session.get_service("ALBasicAwareness")
        self.speaking_movement = self._session.get_service("ALBasicAwareness")
        self.dcm = self._session.get_dcm_wrapper
        self.mem = self._session.get_service("ALMemory")
        self.tablet = self._session.get_service("ALTabletService")
        self.diagnosis = self._session.get_service("ALDiagnosis")
        self.face_detection = self._session.get_service("ALFaceDetection")
        self.face_characteristics = self._session.get_service("ALFaceCharacteristics")
        self.people_detection = self._session.get_service("ALPeoplePerception")
        self.speech_recognition = self._session.get_service("ALSpeechRecognition")
        self.video_device = self._session.get_service("ALVideoDevice")
        self.tts = self._session.get_service("ALTextToSpeech")
        self.tts.setLanguage("English")

        self.speaking_movement = self._session.get_service("ALSpeakingMovement")
        self.animated_speech = self._session.get_service("ALAnimatedSpeech")

        rospy.loginfo("Created all services")


if __name__ == "__main__":
    try:
        rospy.init_node("proxy", anonymous=True)
        p = Proxy()
        rospy.loginfo("Initialized proxy as p")
    except Exception as e:
        rospy.logerr("Could not initialize proxy: %s", e)
