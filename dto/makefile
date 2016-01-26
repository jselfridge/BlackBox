
##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
##  makefile - BLACKBOX
##  Justin M Selfridge
##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

all : BLACKBOX-00A0.dtbo

%.dtbo : %.dts
	dtc -O dtb -o $@ -b 0 -@ $< 

install :
	cp -v *.dtbo /lib/firmware

clean : 
	rm *.dtbo



