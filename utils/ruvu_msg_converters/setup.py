# Copyright 2020 RUVU Robotics B.V.

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ruvu_msg_converters'],
    scripts=[],
    package_dir={'': 'src'}
)

setup(**d)
