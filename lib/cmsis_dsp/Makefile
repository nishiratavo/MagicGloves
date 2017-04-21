CC         = arm-none-eabi-gcc
SRC        = $(wildcard */*.c)
OBJ        = $(SRC:.c=.o)
OBJCOPY    = arm-none-eabi-objcopy
OBJDUMP    = arm-none-eabi-objdump
OPTIMIZE   = -O3
CFLAGS     = -Wall -mcpu=cortex-m4 -mtune=cortex-m4 -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16  -mthumb -I ../../inc/ -D ARM_MATH_CM4 -D __FPU_PRESENT=1 $(OPTIMIZE)
LIBFILE		= ../arm_math.a

all: $(LIBFILE)

$(LIBFILE): $(OBJ)
	arm-none-eabi-ar rcs $@ $(OBJ)

.c.o:
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJ) $(LIBFILE)
