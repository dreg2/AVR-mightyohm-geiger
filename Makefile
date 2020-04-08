# project
TARGET     = geiger
SOURCES    = $(TARGET).c
INCLUDES   =
I_DIRS     =
DEFINES    =
VPATH      =
L_SOURCES  =
L_LIBS     =
L_DIRS     =

# stdio printf options: float, minimal, normal
#L_PRINTF   = -Wl,-u,vfprintf -lprintf_flt -lm
#L_PRINTF   = -Wl,-u,vfprintf -lprintf_min
L_PRINTF   =


# target specifics
AVR_DEVICE = attiny2313
AVR_CLOCK  = 8000000
AVR_PGMR   = arduino
AVR_PORT   = /dev/ttyACM0
AVR_BAUD   = 19200
AVR_LFUSE  = 0xEE
AVR_HFUSE  = 0xDD
AVR_EFUSE  = 0xFF

UART_BAUD  = 9600
UART_PORT  = /dev/ttyUSB0


# files
ELF        = $(TARGET).elf
HEX        = $(TARGET).hex
LIB        = lib$(TARGET).a
OBJS       = $(SOURCES:.c=.o)
OBJS      := $(OBJS:.S=.o)
L_OBJS     = $(L_SOURCES:.c=.o)
L_OBJS    := $(L_OBJS:.S=.o)

# compiler options
DEFINES   += -D F_CPU=$(AVR_CLOCK) -D BAUD=$(UART_BAUD)
C_FLAGS    = -mmcu=$(AVR_DEVICE) $(DEFINES) $(I_DIRS) \
		-std=gnu11 -Os -W -Wall -pedantic \
		-Wformat-nonliteral -Wcast-align  \
		-Wpointer-arith -Wbad-function-cast \
		-Wstrict-prototypes -Winline -Wundef \
		-Wnested-externs -Wcast-qual -Wshadow \
		-Wconversion -Wwrite-strings \
		-ffloat-store
A_FLAGS    = -mmcu=$(AVR_DEVICE) $(DEFINES)
L_FLAGS    = -mmcu=$(AVR_DEVICE)


# command lines
COMPILE    = avr-gcc $(C_FLAGS)
ASSEMBLE   = avr-gcc $(A_FLAGS)
LINK       = avr-gcc $(L_FLAGS)
OBJCOPY    = avr-objcopy -j .text -j .data -O ihex
OBJDUMP    = avr-objdump -d
SIZE       = avr-size --mcu=$(AVR_DEVICE) -C
READELF    = avr-readelf -a
AR         = avr-ar rcs
AVRDUDE    = avrdude -v -c $(AVR_PGMR) -P $(AVR_PORT) -b $(AVR_BAUD) -p $(AVR_DEVICE)


# symbolic targets
.PHONY: all debug size info flash fuse clean disasm monitor

all: $(HEX)

debug: DEFINES += -D DEBUG
debug: $(HEX)

# command targets
size: $(ELF)
	$(SIZE) $(ELF)

info: $(ELF)
	$(READELF) $(ELF)

flash: $(HEX)
	$(AVRDUDE) -U flash:w:$(HEX):i

fuse:
	$(AVRDUDE) -U hfuse:w:$(AVR_HFUSE):m -U lfuse:w:$(AVR_LFUSE):m -U efuse:w:$(AVR_EFUSE):m

clean:
	rm -f $(LIB) $(HEX) $(ELF) $(OBJS) $(L_OBJS)

disasm: $(ELF)
	$(OBJDUMP) $(ELF)

monitor:
	minicom -D $(AVR_PORT) -b $(UART_BAUD)

# file targets
%.o: %.c $(INCLUDES)
	$(COMPILE) -c $< -o $@

%.o: %.S
	$(ASSEMBLE) -c $< -o $@

$(ELF): $(OBJS) $(L_OBJS)
	$(LINK) $(OBJS) $(L_OBJS) $(L_DIRS) $(L_LIBS) $(L_PRINTF) -o $(ELF)

$(HEX): $(ELF)
	$(OBJCOPY) $(ELF) $(HEX)

#$(LIB): $(OBJS)
#	$(AR) $(LIB) $(OBJS)
