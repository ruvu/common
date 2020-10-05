#!/bin/bash

# Copyright 2020 RUVU Robotics B.V.
_warn() {
  echo -e "\033[33m$1\033[0m"
}

_warn "Sourcing this bash file is depricated, use bash function ruvu-bringup-connect-to-external-ros-master instead"
ruvu-bringup-connect-to-external-ros-master

unset _warn
