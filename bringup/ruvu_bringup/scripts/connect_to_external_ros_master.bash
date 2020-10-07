#!/bin/bash

# Copyright 2020 RUVU Robotics B.V.
_warn() {
  echo -e "\033[33m$1\033[0m"
}

_warn "Sourcing this bash file is deprecated, use bash function ruvu_bringup_connect_to_external_ros_master instead"
ruvu_bringup_connect_to_external_ros_master

unset _warn
