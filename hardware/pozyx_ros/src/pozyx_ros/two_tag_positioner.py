from tag_connection import get_tag_connection
from algorithms.multi_tag_positioner import MultiTagPositioner

from device import DeviceRangerPolling

from two_tag_positioner_types import *


class TwoTagPositioner:
    def __init__(self, tags, anchors, uwb_settings, height_2_5d):
        """
        Two tag positioner interface
        :param tags: List of tags (type=[Tags])
        :param anchors: List of anchors (type=[Anchor]) 
        :param uwb_settings: UWB settings for the tags (type=UWBSettings)
        :param height_2_5d=140: Fixed z height for 2.5 D mode in mm (type=float)
        """
        tag_connections = [get_tag_connection(tag.serial_port, uwb_settings) for tag in tags]

        self._device_ranger_polling = DeviceRangerPolling(
            pozyx_serials={tc.network_id: tc.serial_connection for tc in tag_connections},
            anchor_ids=[anchor.network_id for anchor in anchors]
        )
        self._multitag_positioner = MultiTagPositioner(
            anchor_locations={anchor.network_id: [anchor.position.x, anchor.position.y, anchor.position.z] for anchor in
                              anchors},
            tag_locations={tag.network_id: [tag.position.x, tag.position.y, tag.position.z] for tag in tags},
            height_2_5d=height_2_5d
        )

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
