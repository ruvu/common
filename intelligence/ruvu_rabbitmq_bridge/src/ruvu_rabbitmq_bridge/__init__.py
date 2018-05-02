from roslib.message import get_message_class


def get_topic_name_msg_type_dict(param_value):
    """
    Returns a dictionary that maps topic names to message types
    :param param_value: Parameter dictionary value
    :return: A dictionary that maps topic names to message types
    """
    if not isinstance(param_value, dict):
        raise ValueError("Should be a dictionary of topic names to topic types")

    return {topic_name: get_message_class(msg_type_name) for topic_name, msg_type_name in param_value.iteritems()}
