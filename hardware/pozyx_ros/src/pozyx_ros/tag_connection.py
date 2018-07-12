import os
from collections import namedtuple

import pypozyx

UWBSettings = namedtuple('UWBSettings', 'channel bitrate prf plen gain_db')
TagConnection = namedtuple('TagConnection', 'serial_connection network_id')


def get_tag_connection(tag_serial_port, uwb_settings):
    # Resolve symbolic links
    """
    Sets up the tag connection with the correct uwb settings
    :param tag_serial_port: Serial port of the tag
    :param uwb_settings: UWB Settings for the tag (type=UWBSettings)
    :return: Tag connection (type=_TagConnection)
    """
    resolved_port = os.path.realpath(tag_serial_port)
    serial_connection = pypozyx.PozyxSerial(resolved_port)

    result_set_uwb_settings = serial_connection.setUWBSettings(pypozyx.UWBSettings(*uwb_settings))
    if result_set_uwb_settings != pypozyx.POZYX_SUCCESS:
        raise RuntimeError("Failed to set UWB settings for tag on serial port {}".format(tag.serial_port))

    tag_id_object = pypozyx.NetworkID()
    result_get_network_id = serial_connection.getNetworkId(tag_id_object)

    if result_get_network_id != pypozyx.POZYX_SUCCESS:
        raise RuntimeError("Failed to obtain network id from tag on serial port {}".format(tag.serial_port))

    return TagConnection(serial_connection, int(str(tag_id_object), 0), tag.position)