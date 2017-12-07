# src Makefile
include ../Makefile.common

PATH_HARDWARE=hardware
PATH_MAIN=main
CFLAGSlib+=-Iinclude

all: src

src: mkdir objs app.a

mkdir:
	mkdir -p release

objs:
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/main.o -c $(PATH_MAIN)/main.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/startup.o -c $(PATH_MAIN)/startup.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/timer.o -c $(PATH_HARDWARE)/timer.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/pwm.o -c $(PATH_HARDWARE)/pwm.c
	#$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/exti.o -c $(PATH_HARDWARE)/exti.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/led.o -c $(PATH_HARDWARE)/led.c
	#$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/uart.o -c $(PATH_HARDWARE)/uart.c

app.a:
	$(AR) cr $(PATH_TARGET)/app.a $(PATH_TARGET)/startup.o \
		$(PATH_TARGET)/main.o \
		$(PATH_TARGET)/timer.o \
		$(PATH_TARGET)/pwm.o \
		$(PATH_TARGET)/led.o
	
clean:
	rm -f $(PATH_TARGET)/*
	
