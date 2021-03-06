#!/bin/sh

### BEGIN INIT INFO
# Provides:          autorun
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Should-Start:      $portmap
# Should-Stop:       $portmap
# X-Start-Before:    nis
# X-Stop-After:      nis
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# X-Interactive:     false
# Short-Description: Load the auto run script
# Description:       Load the auto run script
### END INIT INFO


sleep 5
echo BLACKBOX > /sys/devices/platform/bone_capemgr/slots

echo none > /sys/class/leds/beaglebone\:green\:usr0/trigger
echo none > /sys/class/leds/beaglebone\:green\:usr1/trigger
echo none > /sys/class/leds/beaglebone\:green\:usr2/trigger
echo none > /sys/class/leds/beaglebone\:green\:usr3/trigger

echo 1 > /sys/class/leds/beaglebone\:green\:usr0/brightness
echo 1 > /sys/class/leds/beaglebone\:green\:usr1/brightness
echo 1 > /sys/class/leds/beaglebone\:green\:usr2/brightness
echo 1 > /sys/class/leds/beaglebone\:green\:usr3/brightness

sleep 2
cd /root/BlackBox/
./RunBlackBox

exit 0



