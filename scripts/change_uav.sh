#!/bin/bash

if [ -x "$(whereis nvim | awk '{print $2}')" ]; then
 VIM_BIN="$(whereis nvim | awk '{print $2}')"
 HEADLESS="--headless"
elif [ -x "$(whereis vim | awk '{print $2}')" ]; then
 VIM_BIN="$(whereis vim | awk '{print $2}')"
 HEADLESS=""
fi

$VIM_BIN $HEADLESS -E -s -c "%s/uav./$1/g" -c "wqa" -- "../plot_juggler/odometry.xml"
$VIM_BIN $HEADLESS -E -s -c "%s/uav./$1/g" -c "wqa" -- "../rviz/odometry.rviz"
