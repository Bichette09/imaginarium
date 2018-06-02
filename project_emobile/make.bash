#!/bin/bash
cd ..
EXPORT LEDDAR_LIB_DIR=~/leddar_sdk
catkin_make --pkg settings_store hardware_monitor imaginarium_core leddar
