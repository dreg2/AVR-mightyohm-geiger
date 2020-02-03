# Sources
TARGET     = geiger
INCLUDES   =
C_SOURCES  = $(TARGET).c
A_SOURCES  =
LIBS       =
DEFINES    = -D BAUD=$(UART_BAUD)
#DEFINES    = -D BAUD=$(UART_BAUD) -D HIGH_CPM

# target specifics
DEVICE     = attiny2313
CLOCK      = 8000000

UART_PORT  = /dev/ttyUSB0
UART_BAUD  = 9600

AVRD_PGMR  = arduino
AVRD_PORT  = /dev/ttyACM0
AVRD_BAUD  = 19200
AVRD_FLAGS = -v
AVRD_LFUSE = 0xEE
AVRD_HFUSE = 0xDD
AVRD_EFUSE = 0xFF


# stdio printf options: float, minimal, normal
#LIB_PRINTF = -Wl,-u,vfprintf -lprintf_flt -lm
#LIB_PRINTF = -Wl,-u,vfprintf -lprintf_min
LIB_PRINTF =


# files
ELF        = $(TARGET).elf
HEX        = $(TARGET).hex
C_OBJS     = $(C_SOURCES:.c=.o)
A_OBJS     = $(A_SOURCES:.S=.o)
OBJS       = $(C_OBJS) $(A_OBJS)


# compiler options
STANDARD   = -std=gnu11
#TEMPS      = -save-temps
#DEBUG      = -D DEBUG
#OPTIMIZE   = -O2
#INC_DIRS   = -I../../include
#LIB_DIRS   = -L../../lib
C_FLAGS    = $(STANDARD) $(INC_DIRS) $(DEBUG) $(OPTIMIZE) $(TEMPS) $(DEFINES)\
		-Os -D F_CPU=$(CLOCK) -mmcu=$(DEVICE) \
		-W -Wall -pedantic \
		-Wformat-nonliteral -Wcast-align  \
		-Wpointer-arith -Wbad-function-cast \
		-Wstrict-prototypes -Winline -Wundef \
		-Wnested-externs -Wcast-qual -Wshadow \
		-Wconversion -Wwrite-strings \
		-ffloat-store
L_FLAGS    = $(LIB_DIRS) $(LIB_PRINTF) $(LIBS)
#A_FLAGS    = -Wa,-a


# command lines
COMPILE    = avr-gcc $(C_FLAGS)
ASSEMBLE   = avr-gcc $(C_FLAGS) $(A_FLAGS)
AVRDUDE    = avrdude $(AVRD_FLAGS) -c $(AVRD_PGMR) -P $(AVRD_PORT)  -b $(AVRD_BAUD) -p $(DEVICE)
AVROBJCOPY = avr-objcopy -j .text -j .data -O ihex
AVRSIZE    = avr-size -C --mcu=$(DEVICE)
AVRREADELF = avr-readelf -a


# symbolic targets
all:	$(HEX)

# command targets
size:	$(ELF)
	$(AVRSIZE) $(ELF)

info:	$(ELF)
	$(AVRREADELF) $(ELF)

flash:	$(HEX)
	$(AVRDUDE) -U flash:w:$(HEX):i

fuse:
	$(AVRDUDE) -U hfuse:w:$(AVRD_HFUSE):m -U lfuse:w:$(AVRD_LFUSE):m -U efuse:w:$(AVRD_EFUSE):m

install: flash fuse

clean:
	rm -f $(HEX) $(ELF) $(OBJS)

disasm:	$(ELF)
	avr-objdump -d $(ELF)

monitor:
	minicom -D $(UART_PORT) -b $(UART_BAUD)


# file targets
%.o:	%.c $(INCLUDES) Makefile
	$(COMPILE)  -c $< -o $@

%.o:	%.S Makefile
	$(ASSEMBLE) -c $< -o $@

$(ELF): $(OBJS) Makefile
	$(COMPILE) $(OBJS) $(L_FLAGS) -o $(ELF)

$(HEX): $(ELF) Makefile
	$(AVROBJCOPY) $(ELF) $(HEX)

.PHONY: all size info flash fuse install clean disasm monitor
