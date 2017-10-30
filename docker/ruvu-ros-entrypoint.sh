#!/bin/bash
set -e

# setup ros environment
source "/ws/devel/setup.bash"
exec "$@"
