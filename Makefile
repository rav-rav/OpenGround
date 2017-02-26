CFLAGS  = -O1 -g
ASFLAGS = -g
SRC_FILES  = main.c delay.c assert.c led.c lcd.c screen.c console.c font.c debug.c io.c
SRC_FILES += adc.c sound.c timeout.c touch.c cc2500.c spi.c storage.c wdt.c gui.c
SRC_FILES += eeprom.c telemetry.c fifo.c crc16.c config.c
SRC_FILES += multi4in1.c

# add src path
GENERIC_SRCS = $(SRC_FILES:%.c=src/%.c)

TOOLROOT ?= '/cygdrive/d/GNU Tools ARM Embedded/6.2 2016q4/bin'
DFU_UTIL ?= k:\Dokumente\QuadCopter\dfu-util-0.9-win64\dfu-util.exe
MAKEFLAGS+="-j8 "

include Makefile.board

all  : board
