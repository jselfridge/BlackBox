
NET=/etc/network/interfaces
UENV=/boot/uEnv.txt

# Make sure the user is root
if [ `whoami` != 'root' ]; then
    echo "You must be root to install this"
    exit
fi

echo "--- UPDATE DEBIAN ---"
sudo apt-get update

echo "--- INSTALL PACKAGES ---"
sudo apt-get install -y build-essential less emacs git ntp

echo "--- LINUX-IMAGE ---"
sudo apt-get install linux-image-4.1.29-bone-rt-r22
cd /boot/
rm -r *3.8.13*
cd

echo "--- EMACS ---"
echo "(setq backup-directory-alist \`((\".\" . \"~/.emacs.d/backups\")))" >> ~/.emacs
echo "(setq backup-by-copying t)" >> ~/.emacs

echo "--- BASHRC ---"
sed -i -e 's/# export LS_OPTIONS=/export LS_OPTIONS=/g' ~/.bashrc
sed -i -e 's/# eval/eval/g' ~/.bashrc
sed -i -e 's/# alias ls=/alias ls=/g' ~/.bashrc
sed -i -e 's/# alias ll=/alias ll=/g' ~/.bashrc
sed -i -e 's/# alias l=/alias l=/g' ~/.bashrc
echo " " >> ~/.bashrc
echo "SLOTS=/sys/devices/platform/bone_capemgr/slots" >> ~/.bashrc
echo "PINS=/sys/kernel/debug/pinctrl/44e10800.pinmux/pins" >> ~/.bashrc
echo "PINGROUPS=/sys/kernel/debug/pinctrl/44e10800.pinmux/pingroups" >> ~/.bashrc
echo " " >> ~/.bashrc

echo "--- NETWORK ---"
sed -i -e '/# The primary network interface/d'               $NET
sed -i -e '/auto eth0/d'                                     $NET
sed -i -e '/iface eth0 inet dhcp/d'                          $NET
sed -i -e '/# Example to keep MAC address between reboots/d' $NET
sed -i -e '/#hwaddress ether DE:AD:BE:EF:CA:FE/d'            $NET
echo " "                                                  >> $NET
echo "# The primary network interface"                    >> $NET
echo "auto eth0"                                          >> $NET
echo "iface eth0 inet static"                             >> $NET
echo "address 192.168.1.160"                              >> $NET
echo "netmask 255.255.255.0"                              >> $NET
echo "gateway 192.168.1.1"                                >> $NET
echo " "                                                  >> $NET

echo "--- GENERIC OVERLAY ---"
git clone https://github.com/beagleboard/bb.org-overlays
cd ./bb.org-overlays
./dtc-overlay.sh
./install.sh
zcat /proc/config.gz | grep CONFIG_BONE_CAPEMGR
cd
rm -r ./bb.org-overlays/ ./git/

echo "--- STANDARD OVERLAY ---"
#sed -i -e "s/#cape_disable=bone_capemgr.disable_partno=/cape_disable=bone_capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN/g" $UENV
#sed -i -e 's/#dtb=am335x-boneblack-emmc-overlay.dtb/dtb=am335x-boneblack-emmc-overlay.dtb/g' $UENV
#sed -i -e '/cape_universal=enable/ s??#cape_universal=enable?' $UENV
#echo "CAPE=BLACKBOX" > /etc/default/capemgr

echo "--- PRU DRIVERS ---"
git clone https://github.com/beagleboard/am335x_pru_package.git
cp am335x_pru_package/pru_sw/app_loader/include/pruss_intc_mapping.h /usr/include/
cp am335x_pru_package/pru_sw/app_loader/include/prussdrv.h /usr/include/
cd am335x_pru_package/pru_sw/app_loader/interface
CROSS_COMPILE= make
cd ../lib
cp libprussdrv.a /usr/lib/
cp libprussdrvd.a /usr/lib/
cp libprussdrvd.so /usr/lib/
cp libprussdrv.so /usr/lib/
ldconfig
cd ../../utils/pasm_source
source linuxbuild
cd ..
cp pasm /usr/bin/
/usr/bin/pasm
cd
modprobe uio_pruss
rm -r ./am335x_pru_package/

echo "--- BLACKBOX REPO ---"
git clone https://github.com/jselfridge/BlackBox.git
cd ./BlackBox/dto/
make clean
make
make install
cd
mkdir Log
cd

# Install autorun script
#
#
#
#

# Restart the system
sudo reboot

