
##============================================================
##  makefile - BlackBox
##  Justin M Selfridge
##============================================================


EXEC   = RunBox
CC     = gcc
CFLAGS = -Wall -g -c

#LIB    = -lmat -lrot -lrt -lm -lprussdrv -lpthread
#LDIR   = ../Libraries/lib/
#IDIR   = ../Libraries/inc/

#SRC   := $(shell cd src; ls -F | grep ".c" )
#CNAME := $(patsubst %.c, %, $(SRC) )
#OBJ   := $(foreach o, $(CNAME), obj/$(o).o )

#PRU   := $(shell cd pru; ls -F | grep ".p" )
#PNAME := $(patsubst %.p, %, $(PRU) )
#BIN   := $(foreach b, $(PNAME), bin/$(b).bin )

#MPU    = mpu/inv_glue.o \
         mpu/inv_mpu.o  \
         mpu/inv_mpu_dmp_motion_driver.o


#all : $(EXEC)

#$(EXEC) : $(OBJ) $(BIN) 
#	$(CC) -o $@ $(OBJ) $(MPU) -L$(LDIR) $(LIB)

#obj/%.o : src/%.c inc/%.h
#	$(CC) $(CFLAGS) -I$(IDIR) -Iinc -Impu -o $@ $<

#bin/%.bin : pru/%.p
#	pasm -b $<
#	mv *.bin bin

#clean :
#	rm $(OBJ) $(BIN) $(EXEC)



