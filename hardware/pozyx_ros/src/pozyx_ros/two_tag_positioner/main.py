from __future__ import print_function

import argparse
import json

from two_tag_positioner import Tag, Anchor, Position, UWBSettings, TwoTagPositioner, Input, Velocity2D

if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--tag_serial_ports', help='Serial ports of the tags, e.g. /dev/ttyACM0', nargs="+", type=str,
                        default=['/dev/sensors/pozyx_1-4', '/dev/sensors/pozyx_1-2'])
    parser.add_argument('--tag_positions', help='Positions of the tags w.r.t the robot"s coordinate frame in mm',
                        nargs="+", type=str,
                        default=[
                            '{"x": 0, "y": -350, "z": 0}',
                            '{"x": 0, "y": 350, "z": 0}',
                        ])
    parser.add_argument('--anchor_ids', help='Network ids of the pozyx anchors e.g. 0x02', nargs="+", type=int,
                        default=[23602, 12123, 26476, 52659, 43083, 35448])
    parser.add_argument('--anchor_positions', help='Positions of the anchors w.r.t the global coordinate frame in mm',
                        nargs="+", type=str,
                        default=[
                            '{"x": 9719, "y": 8997, "z": 4099}',
                            '{"x": 1, "y": 0, "z": 4141}',
                            '{"x": 0, "y": 8951, "z": 4092}',
                            '{"x": 9735, "y": 18694, "z": 4109}',
                            '{"x": 9720, "y": 465, "z": 4129}',
                            '{"x": 5, "y": 18658, "z": 4115}'
                        ])
    parser.add_argument('--uwb_settings', help='UWB Settings for the tags.', type=str,
                        default='{"channel": 5, "bitrate": 2, "prf": 2, "plen": 4, "gain_db": 30.0}')
    args = parser.parse_args()

    tags = [Tag(serial_port=serial_port, position=Position(**json.loads(tag_position)))
            for (serial_port, tag_position) in zip(args.tag_serial_ports, args.tag_positions)]
    anchors = [Anchor(network_id=anchor_id, position=Position(**json.loads(anchor_position)))
               for (anchor_id, anchor_position) in zip(args.anchor_ids, args.anchor_positions)]
    uwb_settings = UWBSettings(**json.loads(args.uwb_settings))

    positioner = TwoTagPositioner(tags, anchors, uwb_settings)
    while True:
        try:
            estimated_position = positioner.get_position(
                Input(
                    velocity=Velocity2D(x=0, y=0, yaw=0),
                    covariance=[1e3] * 9
                )
            )
            print("Estimated position", estimated_position)
        except RuntimeError as e:
            print("Runtime error:", e)
