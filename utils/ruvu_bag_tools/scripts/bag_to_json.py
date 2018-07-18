#!/usr/bin/env python
import json
import logging
from argparse import ArgumentParser, FileType

from rosbag import Bag
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

parser = ArgumentParser()
parser.add_argument('--topics', nargs='*')
parser.add_argument('infiles', nargs='+', type=FileType('r'))

args = parser.parse_args()

for infile in args.infiles:
    outfile_name = infile.name + '.json'
    with open(outfile_name, 'a') as outfile:
        if outfile.tell() != 0:
            logging.warning("Skipping processed file %s", outfile_name)
            continue

        logger.info("Processing %s", outfile_name)
        with Bag(infile) as bag:
            data = []

            for topic, msg, t in bag.read_messages(topics=args.topics):
                data.append({
                    "topic": topic,
                    "msg": convert_ros_message_to_dictionary(msg),
                    "timestamp": t.to_sec(),
                })

            if not data:
                logger.error('Processed %d messages', len(data))
            else:
                json.dump(data, outfile, separators=(',', ':'))
                logger.info('Processed %d messages', len(data))
