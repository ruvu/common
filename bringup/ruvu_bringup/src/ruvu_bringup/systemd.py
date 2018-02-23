TEMPLATE = """[Unit]
Description=%s

[Install]
WantedBy=multi-user.target

[Service]
Restart=on-abort
User=%s
ExecStart=/bin/bash -c "source %s && roslaunch %s --wait"
"""


def template(description, user, environment_bash, launch_file):
    return  TEMPLATE % (description, user, environment_bash, launch_file)