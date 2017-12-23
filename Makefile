-include .prjconf.mk
PRJ_NAME   = MagicGlove
CC         = arm-none-eabi-gcc
SRC        = $(wildcard src/*.c)
ASRC       = $(wildcard src/*.s)
OBJ        = $(SRC:.c=.o) $(ASRC:.s=.o)
LIBDIR     = lib
LIBS       = $(LIBDIR)/arm_math.a
DEPS       = $(wildcard src/*.h) $(wildcard inc/*.h)
OBJCOPY    = arm-none-eabi-objcopy
OBJDUMP    = arm-none-eabi-objdump
PROGRAMMER = openocd
DEVICE     = STM32F407xx
OPTIMIZE   = -O3
DBGFLAGS   = -Og
LDSCRIPT   = stm32f407vgtx.ld
CFLAGS     = -g3 -Wall -mcpu=cortex-m4 -mtune=cortex-M4 -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16  -mthumb -D ARM_MATH_CM4 -I inc/ -D $(DEVICE)
ASFLAGS    =  $(CFLAGS)
LDFLAGS    = -T $(LDSCRIPT) -Wl,--gc-sections --specs=nano.specs -u _printf_float  --specs=rdimon.specs

.PHONY: all debug burn clean config rst fast fastd libs

ifeq ($(CONF), true)

all: CFLAGS += $(OPTIMIZE)
all: libs $(PRJ_NAME).elf

debug: CFLAGS += $(DBGFLAGS)
debug: libs $(PRJ_NAME).elf

burn:
	$(PROGRAMMER) $(PGFLAGS)

else

all: config
debug: config
burn: config

endif

$(PRJ_NAME).elf: $(OBJ) $(LIBS)
	$(CC) $(CFLAGS) $(OBJ) $(LIBS) -o $@ $(LDFLAGS)
	arm-none-eabi-size $(PRJ_NAME).elf

%.o: %.c $(DEPS)
	$(CC) -c $(CFLAGS) $< -o $@

%.o: %.s $(DEPS)
	$(CC) -c $(ASFLAGS) $< -o $@

$(LIBS):
	$(MAKE) -C $(LIBDIR)

libs:
	$(MAKE) -C $(LIBDIR)

clean:
	$(MAKE) clean -C lib
	rm -f $(OBJ) $(OBJ:.c=.d) *.map *.elf *.hex

config:
	./config

rst:

fast: all burn

fastd: debug burn
