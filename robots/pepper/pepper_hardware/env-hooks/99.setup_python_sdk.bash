PACKAGE_PATH=`rospack find pepper_hardware`

SDK_PATH=$PACKAGE_PATH/sdk/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages
export PYTHONPATH=$PYTHONPATH:$SDK_PATH
