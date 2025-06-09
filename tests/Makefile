CC ?= gcc
CFLAGS += -I../schichor_program/Core/Inc -I.

all: test_fonts

# Single source compilation since fonts.c is included in test_fonts.c

test_fonts: test_fonts.c stm32f4xx_hal.h
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f test_fonts
