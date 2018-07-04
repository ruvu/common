import os
from collections import namedtuple

import pypozyx

from algorithms.multi_tag_positioner import MultiTagPositioner
from device import DeviceRangerPolling

# External interface
Position = namedtuple('Position', 'x y z')  # in mm
Velocity2D = namedtuple('Velocity2D', 'x y yaw')  # in mm/s and rad/s
Orientation = namedtuple('Orientation', 'pitch roll yaw')  # in rad
UWBSettings = namedtuple('UWBSettings', 'channel bitrate prf plen gain_db')
Tag = namedtuple('Tag', 'serial_port position')  # Position w.r.t. robot coordinate frame
Anchor = namedtuple('Anchor', 'network_id position')  # Position w.r.t. global coordinate frame
Input = namedtuple('Input', 'velocity covariance')
Output = namedtuple('Output', 'position orientation covariance')

_TagConnection = namedtuple('TagConnection', 'serial_connection network_id position')


class TwoTagPositioner:
    def __init__(self, tags, anchors, uwb_settings):
        """
        Two tag positioner interface
        :param tags: List of tags (type=[Tags])
        :param anchors: List of anchors (type=[Anchor]) 
        :param uwb_settings: UWB settings for the tags (type=UWBSettings)
        """
        tag_connections = [self._get_tag_connection(tag, uwb_settings) for tag in tags]

        self._device_ranger_polling = DeviceRangerPolling(
            pozyx_serials={tc.network_id: tc.serial_connection for tc in tag_connections},
            anchor_ids=[anchor.network_id for anchor in anchors]
        )
        self._multitag_positioner = MultiTagPositioner(
            anchor_locations={anchor.network_id: [anchor.position.x, anchor.position.y, anchor.position.z] for anchor in
                              anchors},
            tag_locations={tc.network_id: [tc.position.x, tc.position.y, tc.position.z] for tc in tag_connections}
        )

    @staticmethod
    def _get_tag_connection(tag, uwb_settings):
        # Resolve symbolic links
        """
        Sets up the tag connection with the correct uwb settings
        :param tag: Tag information (type=Tag)
        :param uwb_settings: UWB Settings for the tag (type=UWBSettings)
        :return: Tag connection (type=_TagConnection)
        """
        serial_connection = pypozyx.PozyxSerial(os.path.realpath(tag.serial_port))

        result_set_uwb_settings = serial_connection.setUWBSettings(pypozyx.UWBSettings(*uwb_settings))
        if result_set_uwb_settings != pypozyx.POZYX_SUCCESS:
            raise RuntimeError("Failed to set UWB settings for tag on serial port {}".format(tag.serial_port))

        tag_id_object = pypozyx.NetworkID()
        result_get_network_id = serial_connection.getNetworkId(tag_id_object)

        if result_get_network_id != pypozyx.POZYX_SUCCESS:
            raise RuntimeError("Failed to obtain network id from tag on serial port {}".format(tag.serial_port))

        return _TagConnection(serial_connection, int(str(tag_id_object), 0), tag.position)

    def get_position(self, input):
        """
        Returns the estimated position
        :param input: System model input for the positioner (type=Input)
        :return: Estimated position (type=Output)
        """
        ranges = self._device_ranger_polling.get_ranges()
        positions = self._multitag_positioner.get_positions(input, ranges)

        if not positions:
            raise RuntimeError("Multitag positioner returned empty list!")

        position = positions[-1]
        if not position["success"]:
            raise RuntimeError("Multitag positioning unsuccessful!")

        return Output(
            position=Position(
                x=position["coordinates"]["x"],
                y=position["coordinates"]["y"],
                z=position["coordinates"]["z"]
            ),
            orientation=Orientation(
                pitch=position["orientation"]["pitch"],
                roll=position["orientation"]["roll"],
                yaw=position["orientation"]["yaw"]
            ),
            covariance=position["covariance"]
        )
