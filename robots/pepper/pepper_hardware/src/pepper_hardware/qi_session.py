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

import qi
import rospy
import sys


class DCMWrapper:
    def __init__(self, dcm):
        self._dcm = dcm

    def set(self, key, value):
        self._dcm.set([key, "ClearAll", [[value]]])


class MemoryWrapper:
    def __init__(self, mem):
        self._mem = mem
        self._subscribers = []  # Subscriber, Service, event tuple

    def subscribe(self, service, event, callback_function, clear_service_subscribers=True):
        rospy.loginfo("Subscribing to {} ...".format(event))

        if clear_service_subscribers:
            for name, period, precision in service.getSubscribersInfo():
                service.unsubscribe(name)

        service.subscribe(rospy.get_name())

        subscriber = self._mem.subscriber(event)
        id = subscriber.signal.connect(callback_function)

        self._subscribers.append((subscriber, service, event, id))

        rospy.loginfo("Subscribed to {}".format(event))

    def unsubscribe(self, service, event):
        [sub.signal.disconnect(id) for (sub, srv, evnt, id) in self._subscribers if evnt == event]
        self._subscribers = [(sub, srv, evnt, id) for (sub, srv, evnt, id) in self._subscribers if event != evnt]
        service.unsubscribe(rospy.get_name())

    def get(self, key):
        return self._mem.getData(key)


class QiSession:

    def __init__(self):
        """
        Proxy class to interface with naoqi running on the robot
        """
        self.pepper_ip = rospy.get_param("~ip", "pepper.local")
        pepper_port = rospy.get_param("~port", 9559)
        url = "tcp://{}:{}".format(self.pepper_ip, pepper_port)

        rospy.loginfo("Connecting to Pepper @ {} ..".format(url))

        self._session = qi.Session()
        self._session.connect(url)

    def get_service(self, service_name):
        rospy.loginfo("Requesting service {} ...".format(service_name))

        try:
            srv = self._session.service(service_name)
        except RuntimeError as e:
            rospy.logerr(e)
            rospy.signal_shutdown(e)
            sys.exit(1)

        rospy.loginfo("Service {} created".format(service_name))

        return srv

    def get_dcm_wrapper(self):
        return DCMWrapper(self.get_service("DCM"))

    def get_memory_wrapper(self):
        return MemoryWrapper(self.get_service("ALMemory"))
