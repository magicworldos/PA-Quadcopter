MOD_NAME = io_stm32

all:
	gcc -I../../include/ -I./ -std=gnu11 -fPIC -shared -o lib$(MOD_NAME).so	\
		$(MOD_NAME).c 
	mv lib$(MOD_NAME).so ../../lib/
clean:
	rm ../../lib/lib$(MOD_NAME).so