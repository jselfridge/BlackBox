
##============================================================
##  makefile - BlackBox
##  Justin M Selfridge
##============================================================


EXEC   = RunBlackBox
CC     = gcc
CFLAGS = -Wall -g -c -fstack-check -fstack-usage -O3

LIB    = -lm -lrt -lpthread -lprussdrv
#LDIR   = ../Libraries/lib/
#IDIR   = ../Libraries/inc/

SRC   := $(shell cd src; ls -F | grep ".c" )
CNAME := $(patsubst %.c, %, $(SRC) )
OBJ   := $(foreach o, $(CNAME), obj/$(o).o )

PRU   := $(shell cd pru; ls -F | grep ".p" )
PNAME := $(patsubst %.p, %, $(PRU) )
BIN   := $(foreach b, $(PNAME), bin/$(b).bin )

#mpu/inv_glue.o
#MPU    = mpu/inv_mpu.o  \
         mpu/inv_mpu_dmp_motion_driver.o

all : $(EXEC)

$(EXEC) : $(OBJ) $(BIN)
	$(CC) -o $@ $(OBJ) $(LIB)  # -L$(LDIR) $(MPU)

obj/%.o : src/%.c inc/%.h
	$(CC) $(CFLAGS) -Iinc  -o $@ $<  # -I$(IDIR) -Impu

bin/%.bin : pru/%.p
	pasm -b $<
	mv *.bin bin

clean :
	rm  $(EXEC) $(OBJ) $(BIN)



