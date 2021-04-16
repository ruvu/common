#!/usr/bin/env python

# Copyright 2020 RUVU Robotics B.V.

import os
import platform
import sys
import unittest
from queue import Queue
from json import dumps
from pika import BlockingConnection, ConnectionParameters

import rospy
import rostest
from std_msgs.msg import String

PKG = 'ruvu_rabbitmq_bridge'
NAME = 'test_publishers'


class TestClient(unittest.TestCase):
    def __init__(self, *args):
        super(TestClient, self).__init__(*args)

    @classmethod
    def setUpClass(cls):
        rospy.init_node(NAME, disable_signals=True)

    def setUp(self):
        self.message_queue = Queue()

    def test_client(self):
        # create a subscriber and wait till the bridge is online
        sub = rospy.Subscriber('string_from_rabbitmq', String, self.string_cb)
        rospy.logerr('waiting for %s', sub.name)
        while not rospy.is_shutdown() and sub.get_num_connections() != 1:
            rospy.sleep(0.1)

        # publish somestring with pika
        host = os.environ.get('RABBITMQ_HOST', 'localhost')
        queue = platform.node() + '/string_from_rabbitmq'
        connection = BlockingConnection(ConnectionParameters(host))
        channel = connection.channel()
        channel.basic_publish(exchange='Test.Publishers', routing_key=queue, body=dumps({'data': 'somestring'}))
        connection.close()

        # check if the string is received in a ROS message
        msg = self.message_queue.get(timeout=30)  # with a timeout, get somehow accepts SIGINT
        self.assertIsNotNone(msg)

    def string_cb(self, msg):
        rospy.loginfo('received a string: %s', msg.data)
        self.message_queue.put(msg)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestClient, sys.argv)
