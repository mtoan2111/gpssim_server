CC = gcc
CFLAGS = -g -O3 -lm -lpthread

all: server.o serial.o
	$(CC) server.o serial.o $(CFLAGS) -o server
	rm -f *.o
clean:
	rm -f *.o
	rm -f server

