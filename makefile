MCU=at90can128
F_CPU=16000000
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-std=c99 -Wall -g -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -I.
TARGET=EVMS_Monitor3
SRCS=EVMS_Monitor3.c can_drv.c can_lib.c Touchscreen.c

all:
	${CC} ${CFLAGS} -o ${TARGET}.bin ${SRCS}
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.bin ${TARGET}.hex

flash:
	avrdude -p ${MCU} -c usbtiny -U flash:w:${TARGET}.hex:i
	
clean:
	rm -f *.bin *.hex

check:
	avrdude -p ${MCU} -c usbtiny

extract:
	avrdude -p ${MCU} -c usbtiny -U flash:r:EVMS_Monitor3_backup.hex:i