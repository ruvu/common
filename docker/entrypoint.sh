#!/bin/bash
set -e

# setup the ruvu environment
source ~/.ruvu/setup.bash
exec "$@"
