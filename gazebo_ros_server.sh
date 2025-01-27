#!/bin/bash

# Call: ./gazebo_server world.world

DIR="$( cd "$( dirname "$0" )" >/dev/null && pwd )"
echo $DIR
export GAZEBO_MODEL_PATH=$DIR/models/:$GAZEBO_MODEL_PATH 

#export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-10/plugins/:$GAZEBO_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH=$DIR/marble_contact_plugin/build:$GAZEBO_PLUGIN_PATH
export GAZEBO_RESOURCE_PATH=${DIR}:$GAZEBO_RESOURCE_PATH}

roslaunch verbose:=true "$@"

