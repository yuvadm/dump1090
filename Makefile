CFLAGS=-O2 -g -Wall -W `pkg-config --cflags librtlsdr`
LIBS=`pkg-config --libs librtlsdr` -lpthread -lm
CC=gcc
PROGNAME=dump1090

all: dump1090

cpr.o: cpr.c
	$(CC) $(CFLAGS) cpr.c -c -o cpr.o

dump1090.o: dump1090.c
	$(CC) $(CFLAGS) dump1090.c -c -o dump1090.o

dump1090: cpr.o dump1090.o
	$(CC) -g -o dump1090 dump1090.o cpr.o $(LIBS)

clean:
	rm -f *.o dump1090
