ASM_FLAGS	= -p P16F690
PK2_FLAGS	= -P PIC16F690

PK2_CMD		= pk2cmd $(PK2_FLAGS)

TARGET		= main.hex

%.hex:	%.asm
	gpasm $(ASM_FLAGS) $<

all:	$(TARGET)


test:	all upload

clean:
	rm -f *.o *.cod *.hex *.lst *.err

upload:
	$(PK2_CMD) -M -F $(TARGET)

power_up:
	$(PK2_CMD) -T

power_down:
	$(PK2_CMD) -W

.PHONY:	all clean upload power_up power_down
