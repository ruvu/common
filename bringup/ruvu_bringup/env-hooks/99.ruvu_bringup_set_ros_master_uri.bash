function ruvu-bringup-set-ros-master-uri {
  _error() {
      echo -e "\033[31m$1\033[0m"
  }

  if [ -z "$1" ]
  then
      _error "Please provide a [ros_master_uri, e.g. http://192.168.1.2:11311] as first argument"
  else
      echo "=> [ruvu_bringup] Setting ROS_MASTER_URI to $1"
      export ROS_MASTER_URI=$1
  fi

  unset _error
  unset _INTERFACE_IP
}
