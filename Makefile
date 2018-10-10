CC = gcc
CFLAGS ?= -g -O3 -lm -lpthread
CFLAGSLD ?= -g -O3 -lpthread -c
OBJS = serial server

ifdef DEBUG
CFLAGS += -DDEBUG=1
endif

.PHONY: all clean
all: serial.o server.o
	@$(CC) server.o serial.o $(CFLAGS) -o server
	@rm -f *.o
	@echo "Done"
serial.o: serial.c
	@$(CC) $(CFLAGS) -c $<

server.o: server.c
	@$(CC) $(CFLAGS) -c $<

clean: 
	@rm -f *.o
	@rm -f server
	@echo "Clean all executable file"

