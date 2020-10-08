# Copyright 2020 RUVU Robotics B.V.

TEMPLATE = """[Unit]
Description=%s

[Install]
WantedBy=multi-user.target

[Service]
Restart=on-abort
User=%s
Environment=PATH=~/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin
ExecStart=/bin/bash -c "source %s && roslaunch %s %s --wait"
"""


def template(description, user, environment_bash, pkg, launch_file):
    return TEMPLATE % (description, user, environment_bash, pkg, launch_file)
