MOD_NAME = automatic

all:
	gcc -I../../include/ -I./ -std=gnu11 -fPIC -shared -o lib$(MOD_NAME).so	\
		 $(MOD_NAME).c
		 
	mv lib$(MOD_NAME).so ../../lib/
	
clean:
	rm ../../lib/lib$(MOD_NAME).so