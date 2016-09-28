MOD_NAME = motor

all:
	gcc -I../../include/ -I./ -fPIC -shared -o lib$(MOD_NAME).so	\
		$(MOD_NAME).c 
	mv lib$(MOD_NAME).so ../../lib/
clean:
	rm ../../lib/lib$(MOD_NAME).so