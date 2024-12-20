#!/usr/bin/env python

# Copyright 2020 RUVU Robotics B.V.

from __future__ import print_function
import os.path
import pwd
from argparse import ArgumentParser

from ruvu_bringup.systemd import template


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)
    else:
        return arg


def is_valid_user(parser, arg):
    if arg not in pwd.getpwall():
        parser.error("The user %s does not exist!" % arg)
    else:
        return arg


def get_package_name(launch_file):
    launch_file_folders = launch_file.split("/")[:-1]
    while launch_file_folders:
        package_xml = "/".join(launch_file_folders) + "/package.xml"
        if os.path.exists(package_xml):
            return launch_file_folders[-1]
        launch_file_folders = launch_file_folders[:-1]
    return "unknown package"


parser = ArgumentParser(description="systemd_service_from_launch_file")
parser.add_argument("launch_file", help="Launch file to generate a service from",
                    type=lambda x: is_valid_file(parser, x))
parser.add_argument("user", help="The user that should execute the launch file", type=str)
parser.add_argument("environment_bash", help="Bash environment to run before", type=lambda x: is_valid_file(parser, x))
args = parser.parse_args()

launch_file = os.path.abspath(args.launch_file)
environment_bash = os.path.abspath(args.environment_bash)
user = args.user
description = "Launch file %s of package %s for user %s" % \
              (os.path.basename(launch_file), get_package_name(launch_file), user)

print(template(description, user, environment_bash, launch_file))
