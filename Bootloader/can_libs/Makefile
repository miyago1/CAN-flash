CC=avr-gcc
MCU=atmega328p
F_CPU = 16000000L
CFLAGS=-std=c99 -Wall -g -Os -mmcu=${MCU} -I. -DF_CPU=${F_CPU}


mh_can_spi.o: mh_can.c mh_can.h mh_j1939.c mh_j1939.h flash_utils.c flash_utils.h
	${CC} -Os -c mh_can.c mh_j1939.c flash_utils.c ${CFLAGS}

clean:
	rm *.o

