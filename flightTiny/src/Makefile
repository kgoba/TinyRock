PROGRAMMER = usbasp
F_CPU 	= 8000000
MCU 	= attiny85
LFUSE 	= 0xE2
HFUSE 	= 0xDD
XFUSE 	= 0xFF

CC = avr-gcc
LD = avr-gcc
OBJDUMP = avr-objdump
OBJCOPY = avr-objcopy

TARGET 	= rfid
SRC 	= rfid.c
CFLAGS = -g -mmcu=$(MCU) -Os -DF_CPU=$(F_CPU)
INCLUDES = 
LFLAGS = -g -mmcu=$(MCU)
LIBS = 

OBJ = $(SRC:.c=.o)

all: $(TARGET).hex $(TARGET).lst

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -j .text -j .data -O ihex $(TARGET).elf $(TARGET).hex

$(TARGET).lst: $(TARGET).elf
	$(OBJDUMP) -h -S $(TARGET).elf >$(TARGET).lst

$(TARGET).elf: $(OBJ)
	$(LD) -o $(TARGET).elf $(OBJ) $(LFLAGS) $(LIBS)

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

fuse:
	avrdude -p $(MCU) -c $(PROGRAMMER) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(XFUSE):m

program: $(TARGET).hex
	avrdude -p $(MCU) -c $(PROGRAMMER) -U flash:w:$(TARGET).hex

