include ../Makefile.common

PROGRAM=main
CC=gcc
LIBS=-lm
FLAGS=
INCLUDES=-Iinclude

PATH_MAIN=main

all: linux

linux:
	mkdir -p $(PATH_TARGET)
	$(CC) $(LIBS) $(FLAGS) $(INCLUDES) -o $(PATH_TARGET)/$(PROGRAM) $(PATH_MAIN)/main.c
	
clean:
	rm -f $(PATH_TARGET)/*