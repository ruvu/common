function ruvu-bringup-set-ros-ip-from-network-interface {
  _error() {
      echo -e "\033[31m$1\033[0m"
  }

  if [ -z "$1" ]
  then
      _error "Please provide a [network_interface, e.g. eno1, eth0] as first argument"
  else
      _INTERFACE_IP="$(ip -json -4 addr show "$1" 2> /dev/null | jq '.[].addr_info[0].local' -r | grep -v null)"

      # Fallback for distro's older then 18.04
      if [[ ! $_INTERFACE_IP ]]
      then
          _INTERFACE_IP="$(/sbin/ifconfig $1 2>/dev/null | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}' 2> /dev/null)"
      fi

      if [ $_INTERFACE_IP ]
      then
          echo "=> [ruvu_bringup] Setting ROS_IP to $_INTERFACE_IP ($1)"
          export ROS_IP=$_INTERFACE_IP
      else
          _error "Failed to set ROS_IP. Interface $1 does not exist or does not have an IP."
      fi
  fi

  unset _error
  unset _INTERFACE_IP
}
