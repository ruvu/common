import json

from pypozyx import *


class DevicePositioner(object):
    def __init__(self, pozyx_serials, anchor_locations, **kwargs):
        self.pozyx_serials = pozyx_serials
        self.set_anchors_on_devices(anchor_locations)
        # settings
        self.algorithm = kwargs.get('algorithm', POZYX_POS_ALG_UWB_ONLY)
        self.dimension = kwargs.get('dimension', POZYX_3D)
        self.height = kwargs.get('height', 1000)
        self.tag_coordinates = None

    def set_anchors_on_devices(self, anchor_locations):
        # define anchor DeviceCoordinates
        anchor_device_coordinates = {}
        for anchor_id in anchor_locations:
            anchor_device_coordinates[anchor_id] = DeviceCoordinates(anchor_id, 1,
                                                                     Coordinates(anchor_locations[anchor_id][0],
                                                                                 anchor_locations[anchor_id][1],
                                                                                 anchor_locations[anchor_id][2]))
        # for each tag device clear and then add anchor DeviceCoordinates one by one
        for tag_id in self.pozyx_serials:
            status = self.pozyx_serials[tag_id].clearDevices()
            for anchor_id in anchor_device_coordinates:
                status &= self.pozyx_serials[tag_id].addDevice(anchor_device_coordinates[anchor_id])
            if not status == POZYX_SUCCESS:
                print('ALERT: setting anchors on the tags failed (' + str(tag_id) + ')')

    def get_positions(self, positioning_input):
        self.tag_coordinates = {}
        for tag_id in self.pozyx_serials:
            position = Coordinates()
            status = self.pozyx_serials[tag_id].doPositioning(position, self.dimension, self.height, self.algorithm)
            if status == POZYX_SUCCESS:
                if position.x == position.y == position.z == 0:
                    self.tag_coordinates[tag_id] = {'success': False,
                                                    'error_code': 'DEVICE_POSITIONING_ZERO'}
                else:
                    self.tag_coordinates[tag_id] = {'success': True,
                                                    'coordinates': {'x': position.x, 'y': position.y, 'z': position.z}}
            else:
                self.tag_coordinates[tag_id] = {'success': False,
                                                'error_code': 'DEVICE_POSITIONING_FAIL'}
        return self.tag_coordinates


class DeviceLogPositioner(object):
    def __init__(self, log_filename):
        self.log_data_list = []
        with open(log_filename) as log_file:
            for log_data in log_file:
                self.log_data_list.append(json.loads(log_data)[0])

    def get_positions(self, positioning_input):
        if len(self.log_data_list):
            new_data = self.log_data_list.pop(0)
            return {int(t_id): new_data[t_id] for t_id in new_data}


class DeviceRanger(object):
    def __init__(self, pozyx_serials, anchor_ids, **kwargs):
        self.pozyx_serials = pozyx_serials
        self.anchor_ids = anchor_ids
        self.tag_ranges = None

    def get_ranges(self, positioning_input):
        self.tag_ranges = {}
        for tag_id in self.pozyx_serials:
            self.tag_ranges[tag_id] = {}
            for anchor_id in self.anchor_ids:
                device_range = DeviceRange()
                status = self.pozyx_serials[tag_id].doRanging(anchor_id, device_range)
                if status == POZYX_SUCCESS:
                    if device_range.distance == 0:
                        self.tag_ranges[tag_id][anchor_id] = {'success': False,
                                                              'error_code': 'DEVICE_RANGING_ZERO'}
                    else:
                        self.tag_ranges[tag_id][anchor_id] = {'success': True,
                                                              'distance': device_range.distance}
                else:
                    self.tag_ranges[tag_id][anchor_id] = {'success': False,
                                                          'error_code': 'DEVICE_RANGING_FAIL'}
        return self.tag_ranges


class DeviceMultiRanger(object):
    def __init__(self, pozyx_serials, anchor_ids, num_ranges, **kwargs):
        self.pozyx_serials = pozyx_serials
        self.anchor_ids = anchor_ids
        self.tag_ranges = None
        self.num_ranges = num_ranges

    def get_ranges(self, positioning_input):
        self.tag_ranges = {}
        for tag_id in self.pozyx_serials:
            self.tag_ranges[tag_id] = {}
            for anchor_id in self.anchor_ids:
                self.tag_ranges[tag_id][anchor_id] = []
                for i in range(self.num_ranges):
                    device_range = DeviceRange()
                    status = self.pozyx_serials[tag_id].doRanging(anchor_id, device_range)
                    if status == POZYX_SUCCESS:
                        if device_range.distance == 0:
                            self.tag_ranges[tag_id][anchor_id].append({'success': False,
                                                                       'error_code': 'DEVICE_RANGING_ZERO'})
                        else:
                            self.tag_ranges[tag_id][anchor_id].append({'success': True,
                                                                       'distance': device_range.distance})
                    else:
                        self.tag_ranges[tag_id][anchor_id].append({'success': False,
                                                                   'error_code': 'DEVICE_RANGING_FAIL'})
        return self.tag_ranges


class DeviceLogRanger(object):
    def __init__(self, log_filename):
        self.log_data_list = []
        with open(log_filename) as log_file:
            for log_data in log_file:
                self.log_data_list.append(json.loads(log_data)[0])

    def get_ranges(self, positioning_input):
        if len(self.log_data_list):
            new_data = self.log_data_list.pop(0)
            return {int(t_id): {int(a_id): new_data[t_id][a_id] for a_id in new_data[t_id]} for t_id in new_data}
        return {}
