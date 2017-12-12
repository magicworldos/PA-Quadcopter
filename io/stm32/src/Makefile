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
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/timer_tick.o -c $(PATH_HARDWARE)/timer_tick.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/pwm_out.o -c $(PATH_HARDWARE)/pwm_out.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/pwm_in.o -c $(PATH_HARDWARE)/pwm_in.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/led_light.o -c $(PATH_HARDWARE)/led_light.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/serial_port.o -c $(PATH_HARDWARE)/serial_port.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/frame_crc.o -c $(PATH_HARDWARE)/frame_crc.c

app.a:
	$(AR) cr $(PATH_TARGET)/app.a $(PATH_TARGET)/startup.o \
		$(PATH_TARGET)/main.o \
		$(PATH_TARGET)/timer_tick.o \
		$(PATH_TARGET)/pwm_out.o \
		$(PATH_TARGET)/pwm_in.o \
		$(PATH_TARGET)/led_light.o \
		$(PATH_TARGET)/serial_port.o \
		$(PATH_TARGET)/frame_crc.o
	
clean:
	rm -f $(PATH_TARGET)/*
	
