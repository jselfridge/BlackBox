
##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
##  makefile - BBB UAV
##  Justin M Selfridge
##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

all : BBB-UAV-00A0.dtbo

%-00A0.dtbo : %.dts
	dtc -O dtb -o $@ -b 0 -@ $<

install :
	cp -v *.dtbo /lib/firmware

clean : 
	rm *.dtbo

