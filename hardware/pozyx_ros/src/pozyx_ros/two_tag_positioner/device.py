import struct
import binascii
import time
from pypozyx import *


RANGE_OFFSET = -125


def get_amount_rangings(p):
    s = 'F,%0.2x,%s,%i\r' % (0xC7, Data([0]).byte_data, 1+1)
    try:
        r = p.serialExchange(s)
    except SerialException:
        return POZYX_FAILURE, []
    ret = binascii.unhexlify(r[2:])
    ret = struct.unpack("<B", ret)
    return POZYX_SUCCESS, ret[0]


def get_rangeinfo(p, amount):
    s = 'F,%0.2x,%s,%i\r' % (0xC7, format(amount, '02x'), 1+(amount*7))
    #print(type(struct.pack("<B", amount).hex()))
    try:
        r = p.serialExchange(s)
    except SerialException:
        return POZYX_FAILURE, []
    ret = binascii.unhexlify(r[2:])
    data = []
    for i in range(0,amount):
        network_id, distance, seq = struct.unpack("<HIB", ret[i*7:i*7+7])
        data.append((network_id, distance, seq))
    return POZYX_SUCCESS, data


class DeviceRangerPolling(object):
    def __init__(self, pozyx_serials, anchor_ids, **kwargs):
        self.pozyx_serials = pozyx_serials
        self.anchor_ids = anchor_ids
        self.tag_anchor_ranges = None
        self.current_index = None
        self.timeout_time = None
        self.timeout = kwargs.get('timeout', 0.2)

    def get_ranges(self):
        self.tag_anchor_ranges = {}
        now_time = time.time()
        while True:
            index_found = False
            now_time = time.time()
            for tag_id in self.pozyx_serials:
                status, amount = get_amount_rangings(self.pozyx_serials[tag_id])
                if status:
                    status, info = get_rangeinfo(self.pozyx_serials[tag_id], amount)
                    if status:
                        for i in info:
                            if not i[0] == 0:
                                if self.current_index is None:
                                    self.timeout_time = now_time + self.timeout
                                    self.current_index = i[2]
                                if i[2] == self.current_index:
                                    index_found = True
                                    if (tag_id, i[0]) not in self.tag_anchor_ranges:
                                        self.tag_anchor_ranges[(tag_id, i[0])] = i[1] + RANGE_OFFSET
            if not index_found or now_time > self.timeout_time:
                self.timeout_time = now_time + self.timeout
                if self.current_index is not None:
                    self.current_index += 1
                    self.current_index %= 256
                if len(self.tag_anchor_ranges):
                    break
        return now_time, self.tag_anchor_ranges
