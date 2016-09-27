MOD_NAME = display

all:
	gcc -I../../include/ -I./ -fPIC -shared $(MOD_NAME).c -o lib$(MOD_NAME).so
	mv lib$(MOD_NAME).so ../../lib/
clean:
	rm ../../lib/lib$(MOD_NAME).so