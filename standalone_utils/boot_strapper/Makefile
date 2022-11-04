#  SPDX-License-Identifier: (BSD-3-Clause)
#  Copyright 2022 NXP
#

ifeq ("$(origin V)", "command line")
  VERBOSE = $(V)
endif
ifndef VERBOSE
  VERBOSE = 0
endif

ifeq ($(VERBOSE),1)
  Q =
else
  Q = @
endif

CROSS_COMPILE	?= arm-none-eabi-
CC		= $(CROSS_COMPILE)gcc
LD		= $(CROSS_COMPILE)ld
objcopy		= $(CROSS_COMPILE)objcopy
objdump		= $(CROSS_COMPILE)objdump

DRIVERS         = i2c

SRCS		= $(wildcard common/*.c) \
		  $(wildcard core/*.S) \
		  $(foreach dir,$(DRIVERS),$(wildcard $(dir)/*.c)) 

INCLUDES        = -I. -Iinclude/ $(foreach dir,$(DRIVERS),-I$(dir))

DEPS            = $(wildcard include/*.h) \
                  $(foreach dir,$(DRIVERS),$(wildcard $(dir)/*.h))

OBJDIR		= bin

OBJS		= $(basename $(SRCS))
OBJS		:= $(notdir $(OBJS))
OBJS		:= $(OBJS:%=%.o)

vpath %.c ./common $(foreach dir,$(DRIVERS),$(dir))
vpath %.S ./core
vpath %.s ./
vpath %.o $(OBJDIR)

LD_FLAGS	= -EL -Map boot_strapper.map
CFLAGS_SOC_ARM	+= -mcpu=cortex-m4 -mthumb -fno-zero-initialized-in-bss -Wall -Wstrict-prototypes -O1 -Werror
CFLAGS		= $(CFLAGS_SOC_ARM) -c -g -nostdlib -fno-builtin -mlittle-endian -fpic
CFLAGS_ASM	= $(CFLAGS_SOC_ARM) -D_ASM -mlittle-endian

all: xyz boot_strapper.elf boot_strapper.bin boot_strapper.asm

xyz:
	$(Q)mkdir -p $(OBJDIR)

boot_strapper.elf: $(OBJS) Makefile ./core/boot_strapper.lds
	@echo "  LD        "$@
	 $(Q)$(LD) -T./core/boot_strapper.lds -o boot_strapper.elf $(OBJDIR)/*.o $(LD_FLAGS)

boot_strapper.bin: boot_strapper.elf
	@echo "  OBJCOPY   "$@
	$(Q)$(objcopy) -O binary $< $@

boot_strapper.asm: boot_strapper.elf
	@echo "  OBJDUMP   "$@
	$(Q)$(objdump) -DS  -j .text -j .ARM.exidx $< > $@

clean:
	@echo "  CLEAN      $(OBJDIR)/ boot_strapper.elf boot_strapper.bin boot_strapper.asm boot_strapper.map"
	$(Q)rm -rf $(OBJDIR)/ boot_strapper.elf boot_strapper.bin boot_strapper.asm boot_strapper.map

distclean:
	@echo "  CLEAN      $(OBJDIR)/ boot_strapper.elf boot_strapper.bin boot_strapper.asm boot_strapper.map"
	$(Q)rm -rf $(OBJDIR)/ boot_strapper.elf boot_strapper.bin boot_strapper.asm boot_strapper.map

%.o: %.c $(DEPS)
	@echo "  CC        "$(OBJDIR)/$@
	$(Q)$(CC) $(CFLAGS) $(INCLUDES) -o $(OBJDIR)/$@ -c $<

%.o: %.S $(DEPS)
	@echo "  AS        "$(OBJDIR)/$@
	$(Q)$(CC) $(CFLAGS_ASM) $(INCLUDES) -o $(OBJDIR)/$@ -c $<
