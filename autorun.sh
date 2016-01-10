#! /bin/sh

echo "Starting BlackBox program"

#sudo echo BLACKBOX > $SLOTS
sudo echo BLACKBOX > /sys/devices/platform/bone_capemgr/slots

cd /root/BlackBox/
./RunBlackBox

exit 0


