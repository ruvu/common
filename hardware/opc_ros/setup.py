#!/usr/bin/env python

# Copyright 2020 RUVU Robotics B.V.

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['opc_ros'],
    package_dir={'': 'src'},
)

setup(**d)
