# Copyright 2020 RUVU Robotics B.V.

from roslib.message import get_message_class, get_service_class
from collections import namedtuple

TopicBridge = namedtuple("TopicBridge", "topic message_type durable exchange")


def get_topic_bridges(param_value):
    """
    :param param_value: Parameter dictionary value
    :return: A list with named tuples
    """
    if not isinstance(param_value, list):
        raise ValueError("Should be a list")

    bridges = []
    for item in param_value:
        if not isinstance(item, dict):
            raise ValueError("Bridge item should be a dictionary")
        try:
            item['message_type'] = get_message_class(item['message_type'])
            bridges.append(TopicBridge(**item))
        except KeyError as e:
            raise ValueError('Missing key {} in topic bridge item'.format(e))

    return bridges


def get_service_bridges(param_value):
    """
    :param param_value: Parameter dictionary value
    :return: A list with named tuples
    """
    if not isinstance(param_value, list):
        raise ValueError("Should be a list")

    bridges = []
    for item in param_value:
        if not isinstance(item, dict):
            raise ValueError("Bridge item should be a dictionary")
        try:
            item['message_type'] = get_service_class(item['message_type'])
            bridges.append(TopicBridge(**item))
        except KeyError as e:
            raise ValueError('Missing key {} in topic bridge item'.format(e))

    return bridges
