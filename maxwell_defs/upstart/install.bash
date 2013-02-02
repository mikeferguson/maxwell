#!/bin/bash

# Run this as root, from the directory containing it!
#
# USAGE: sudo ./install.bash
#  or
# sudo ./install.bash usb0
#
# where usb0 is whatever network interface you want to set the robot
# up for.  wlan0 is the default.

interface=wlan0

if [ $# -gt 0 ]; then
    if [ "$1" != "" ]; then
        interface=$1
    fi
fi

echo "Installing using network interface $interface."

sed "s/wlan0/$interface/g" < maxwell-start > /usr/sbin/maxwell-start
sed "s/wlan0/$interface/g" < maxwell-stop > /usr/sbin/maxwell-stop
sed "s/wlan0/$interface/g" < maxwell.conf > /etc/init/maxwell.conf

