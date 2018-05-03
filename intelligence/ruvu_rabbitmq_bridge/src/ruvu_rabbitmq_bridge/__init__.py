from roslib.message import get_message_class


def get_topic_name_msg_type_dict(param_value):
    """
    Returns a dictionary that maps topic names to message types
    :param param_value: Parameter dictionary value
    :return: A dictionary that maps topic names to message types
    """
    if not isinstance(param_value, list):
        raise ValueError("Should be a list")

    dictionary = {}
    for item in param_value:
        if not isinstance(item, dict) or "topic" not in item or "message_type" not in item:
            raise ValueError("Should be a dictionary with a topic and message_type key")
        dictionary[item['topic']] = get_message_class(item['message_type'])

    return dictionary
