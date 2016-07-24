
##============================================================
##  makefile - BlackBox
##  Justin M Selfridge
##============================================================


EXEC   = RunBlackBox
CC     = gcc
CFLAGS = -Wall -g -c -fstack-check -fstack-usage -O3

LIB    = -lm -lrt -lpthread -lprussdrv -lmat
#LDIR   = ../Libraries/lib/
#IDIR   = ../Libraries/inc/

SRC   := $(shell cd src; ls -F | grep ".c" )
CNAME := $(patsubst %.c, %, $(SRC) )
OBJ   := $(foreach o, $(CNAME), obj/$(o).o )

PRU   := $(shell cd pru; ls -F | grep ".p" )
PNAME := $(patsubst %.p, %, $(PRU) )
BIN   := $(foreach b, $(PNAME), bin/$(b).bin )

DEFS = -DMPU9250


all : $(EXEC)

$(EXEC) : $(OBJ) $(BIN)
	$(CC) -o $@ $(OBJ) -Llib $(LIB)   # -L$(LDIR)

obj/%.o : src/%.c inc/%.h
	$(CC) $(CFLAGS) $(DEFS) -Iinc -Imavlink -o $@ $<

bin/%.bin : pru/%.p
	pasm -b $<
	mv *.bin bin

clean :
	rm  $(EXEC) $(OBJ) $(BIN)



