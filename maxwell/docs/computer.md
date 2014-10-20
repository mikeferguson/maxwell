# Maxwell Computer Config

 * Install 14.04
 * Install ROS:

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
        wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
        sudo apt-get update
        sudo apt-get install ros-indigo-ros-base
        source /opt/ros/indigo/setup.bash
        mkdir -p ~/indigo/src
        cd ~/indigo/src
        git clone git@github.com:mikeferguson/etherbotix_python.git
        git clone git@github.com:mikeferguson/maxwell.git
        cd ~/indigo
        rosdep install --from-paths src --ignore-src --rosdistro indigo -y
        catkin_make

 * At this point, you probably want to add this to the end of your ~/.bashrc:

        source ~/indigo/devel/setup.bash

 * (Optional) [Pair your PS3 Controller](http://wiki.ros.org/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle).
 * (Optional) Setup a PS3 Controller:

        sudo rm /etc/init/bluetooth.conf
        roscd maxwell_defs/config
        sudo cp etc/init/ps3joy.conf /etc/init/ps3joy.conf

 * (Optional) Install upstart scripts. The robot can always be started with
   roslaunch, but upstart allows it to start automatically.

        roscd maxwell_defs/config
        sudo ./install.sh

## Troubleshooting ASUS Xtion

### Updating Firmware
Depending on the age of the ASUS Xtion and the computer you are using, you may have
to update the firmware on the Xtion. To test the Xtion, do the following:

    sudo apt-get install ros-indigo-openni2-launch
    (plug in the xtion now -- or reboot if already plugged in so that
     udev rules are definately up to date)
    roslaunch openni2_launch openni2.launch

If you see something like the following, there is probably an issue:

```
[ INFO] [1413707372.756294067]: No matching device found.... waiting for devices. Reason: openni2_wrapper::OpenNI2Device::OpenNI2Device(const string&) @ /tmp/buildd/ros-indigo-openni2-camera-0.2.1-0trusty-20140921-1358/src/openni2_device.cpp @ 74 : Initialize failed
	Could not open "1d27/0601@2/11": USB transfer timeout!
```

Next, check the dmesg for these lines:

```
[   42.301532] usb 2-2: new high-speed USB device number 6 using xhci_hcd
[   42.321448] usb 2-2: New USB device found, idVendor=1d27, idProduct=0600
[   42.321453] usb 2-2: New USB device strings: Mfr=2, Product=1, SerialNumber=0
[   42.321455] usb 2-2: Product: PrimeSense Device
[   42.321457] usb 2-2: Manufacturer: PrimeSense
[   42.321767] usb 2-2: Not enough bandwidth for new device state.
[   42.321775] usb 2-2: can't set config #1, error -28
```

If you see the "can't set config #1, error -28" line, you almost certainly need
to update the firmware. To do this, you will need a Windows machine. Install
the [Openni2 SDK](http://structure.io/openni) and then grab the updated
[firmware from ASUS](http://www.asus.com/Multimedia/Xtion_PRO_LIVE/HelpDesk_Download/).
Extract the firmware update, and run the program as administrator. After the
update, you should notice that idProduct will now be 0601.

### Blacklisting snd_usb_audio

The newer ASUS firmware tries to use snd_usb_audio as a kernel driver for the
microphones -- this can be problematic if you are on a USB 3.0 bus. Adding

    blacklist snd_usb_audio

to /etc/modprobe.d/blacklist.conf will disable the kernel module.
