import os
from collections import namedtuple

import pypozyx
from algorithms.multi_tag_positioner import MultiTagPositioner

from device import DeviceRangerPolling

# External interface
Position = namedtuple('Position', 'x y z')  # [mm, mm, mm]
Velocity2D = namedtuple('Velocity2D', 'x y yaw')  # [mm/s, mm/s, rad/s]
Orientation = namedtuple('Orientation', 'pitch roll yaw')  # [rad, rad, rad]
UWBSettings = namedtuple('UWBSettings', 'channel bitrate prf plen gain_db')
Tag = namedtuple('Tag', 'serial_port position')  # Position w.r.t. robot coordinate frame
Anchor = namedtuple('Anchor', 'network_id position')  # Position w.r.t. global coordinate frame

# Time in seconds since epoch, Velocity w.r.t. robot coordinate frame and the covariance matrix (size=9) as follows:
# [xx,   xy,   xyaw,                [mm^2/s^2, mm^2/s^2, mm*rad/s^2,
#  yx,   yy,   yyaw,    with units   mm^2/s^2, mm^2/s^2, mm*rad/s^2,
#  yawx, yawy, yawyaw]               mm^2/s^2, mm^2/s^2, rad^2/s^2]
Input = namedtuple('Input', 'current_time velocity_time velocity covariance')

# Position and Orientation w.r.t. global coordinate frame and the covariance matrix (size=36) as follows:
# [xx,     xy,     xz,     xroll,     xpitch,     xyaw,                [mm^2,   mm^2,   mm^2,   rad*mm, rad*mm, rad*mm,
#  yx,     yy,     yz,     yroll,     ypitch,     yyaw,                 mm^2,   mm^2,   mm^2,   rad*mm, rad*mm, rad*mm,
#  zx,     zy,     zz,     zroll,     zpitch,     zyaw,     with units  mm^2,   mm^2,   mm^2,   rad*mm, rad*mm, rad*mm,
#  rollx,  rolly,  rollz,  rollroll,  rollpitch,  rollyaw,              mm*rad, mm*rad, mm*rad, rad^2,  rad^2,  rad^2,
#  pitchx, pitchy, pitchz, pitchroll, pitchpitch, pitchyaw,             mm*rad, mm*rad, mm*rad, rad^2,  rad^2,  rad^2,
#  yawx,   yawy,   yawz,   yawroll,   yawpitch,   yawyaw]               mm*rad, mm*rad, mm*rad, rad^2,  rad^2,  rad^2]
Output = namedtuple('Output', 'position orientation covariance')

_TagConnection = namedtuple('TagConnection', 'serial_connection network_id position')


class TwoTagPositioner:
    def __init__(self, tags, anchors, uwb_settings, height_2_5d):
        """
        Two tag positioner interface
        :param tags: List of tags (type=[Tags])
        :param anchors: List of anchors (type=[Anchor]) 
        :param uwb_settings: UWB settings for the tags (type=UWBSettings)
        :param height_2_5d=140: Fixed z height for 2.5 D mode in mm (type=float)
        """
        tag_connections = [self._get_tag_connection(tag, uwb_settings) for tag in tags]

        self._device_ranger_polling = DeviceRangerPolling(
            pozyx_serials={tc.network_id: tc.serial_connection for tc in tag_connections},
            anchor_ids=[anchor.network_id for anchor in anchors]
        )
        self._multitag_positioner = MultiTagPositioner(
            anchor_locations={anchor.network_id: [anchor.position.x, anchor.position.y, anchor.position.z] for anchor in
                              anchors},
            tag_locations={tc.network_id: [tc.position.x, tc.position.y, tc.position.z] for tc in tag_connections},
            height_2_5d=height_2_5d
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
        resolved_port = os.path.realpath(tag.serial_port)
        serial_connection = pypozyx.PozyxSerial(resolved_port)

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
        timestamp, ranges = self._device_ranger_polling.get_ranges()
        position = self._multitag_positioner.get_position(timestamp, ranges, input)
        if not position["success"] or "fallback" in position:
            raise RuntimeError("Multitag positioning unsuccessful!")

        # Where c
        # [xx,     xy,     xvx,    xvy,      xyaw,      xvyaw,
        #  yx,     yy,     yvx,    yvy,      yyaw,      yvyaw,
        #  vxx,    vxy,    vxvx,   vxvy,     vxyaw,     vxvyaw,
        #  vyx,    vyy,    vyvx,   vyvy,     vyyaw,     vyvyaw,
        #  yawx,   yawy,   yawvx,  yawvy,    yawyaw,    yawvyaw,
        #  vyawx,  vyawy,  vyawvx, vyawvy,   vyawyaw,   vyawvyaw] in [mm, mm, mm/s, mm/s, rad, rad/s]
        c = position["diagnostics"]["covariance"]
        return Output(
            position=Position(
                x=position["state"]["position"][0],
                y=position["state"]["position"][1],
                z=position["state"]["position"][2]
            ),
            orientation=Orientation(
                roll=position["state"]["orientation"][1],
                pitch=position["state"]["orientation"][0],
                yaw=position["state"]["orientation"][2]
            ),
            covariance=[
                c[0][0],   c[0][1], 0, 0, 0,  c[0][4],
                c[1][0],   c[1][1], 0, 0, 0,  c[1][4],
                0,         0,       0, 0, 0,  0,
                0,         0,       0, 0, 0,  0,
                0,         0,       0, 0, 0,  0,
                c[4][0],   c[4][1], 0, 0, 0,  c[4][4]
            ]
        )
