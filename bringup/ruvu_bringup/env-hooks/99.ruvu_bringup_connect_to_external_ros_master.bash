function ruvu_bringup_connect_to_external_ros_master {
  _error() {
    echo -e "\033[31m$1\033[0m"
  }

  if [ -z "$1" ]
  then
    _error "Please provide a [ros_master_uri, e.g. http://192.168.1.2:11311] as argument"
  else
    if [ -z "$2" ]
    then
        _error "Please provide a [network_interface, e.g. eno1, eth0] as argument"
    else
       ruvu_bringup_set_ros_master_uri $1
       ruvu_bringup_set_ros_ip_from_network_interface $2
    fi
  fi

  unset _error
}
