MCU = attiny202
MCU_CLOCK = 5000000UL
CC_OPTS = -g -O2 -Wa,-mgcc-isr
CC = avr-gcc
OBJCOPY = avr-objcopy
TARGETS = key-controller.hex key-controller key-controller.o

.PHONY: all
all: key-controller.hex

.PHONY: clean
clean:
	rm -f $(TARGETS)

key-controller.hex: key-controller
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

key-controller: key-controller.o
	$(CC) -mmcu=$(MCU) $(CC_OPTS) -o $@ $<

key-controller.o: key-controller.c
	$(CC) -mmcu=$(MCU) -DF_CPU=$(MCU_CLOCK) $(CC_OPTS) -c $<
