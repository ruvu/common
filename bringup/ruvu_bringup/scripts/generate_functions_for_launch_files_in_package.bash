#!/bin/bash

# Copyright 2020 RUVU Robotics B.V.

_error() {
    echo -e "\033[31m$1\033[0m"
}

if [ -z "$1" ]
then
    _error "Please provide a [package_name] as first argument"
else
    if [ -z "$2" ]
    then
        _error "Please provide a [prefix] as second argument"
    else
        _PACKAGE_DIR=`rospack find $1 2> /dev/null`
        if [ $? -eq 0 ]
        then
            _LAUNCH_FILES=`find $_PACKAGE_DIR/launch -name '*.launch' 2> /dev/null`
            if [ $? -eq 0 ]
            then
				for launch_file in $_LAUNCH_FILES; do
					launch_file_name=$(echo ${launch_file##*/} | cut -d"." -f1)
					function_name=$2-${launch_file_name//_/-}
					eval "${function_name}() { roslaunch ${launch_file} \$@; }"
				done
            else
                _error "No launch files in package $1"
            fi
        else
            _error "Package $1 not found"
        fi
    fi
fi

unset _error
unset _PACKAGE_DIR
unset _LAUNCH_FILES
