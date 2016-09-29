MOD_NAME = controller

all:
	gcc -I../../include/ -I./ -std=gnu11 -lm -fPIC -shared -o lib$(MOD_NAME).so	\
		 $(MOD_NAME).c
		 
	mv lib$(MOD_NAME).so ../../lib/
clean:
	rm ../../lib/lib$(MOD_NAME).so