/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017-2024 NXP
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

/* Entry Point */
ENTRY(m4_startup)

STACK_SIZE = 0x1000;
HEAP_SIZE = 0x200;

/* Specify the memory areas */

MEMORY
{
  m_startup		(RX)  : ORIGIN = 0x1F800000, LENGTH = 0x00000100
  m_interrupts          (RX)  : ORIGIN = 0x1F800100, LENGTH = 0x00000240
  m_text                (RX)  : ORIGIN = 0x1F800340, LENGTH = 0x000120C0
  m_data                (RW)  : ORIGIN = 0x1F812400, LENGTH = 0x00008800
  m_dtcm		(RW)  : ORIGIN = 0x20000000, LENGTH = 0x0000F000
  br_data               (R)   : ORIGIN = 0x2000F000, LENGTH = 0x00001000
}

/* Define output sections */
SECTIONS
{
  .startup :
  {
    . = ALIGN(4);
    __m_startup_start = .;      /* create a global symbol at m_startup start */
    KEEP(*(.start_up))
    . = ALIGN(4);
    __m_startup_end = .;      /* create a global symbol at m_startup end */
  } > m_startup

  /* The startup code goes first into internal RAM */
  .interrupts :
  {
    __VECTOR_TABLE = .;
    . = ALIGN(4);
    __m_interrupts_start = .;      /* create a global symbol at interrupts start */
    KEEP(*(.isr_vector))     /* Startup code */
    . = ALIGN(4);
    __m_interrupts_end = .;      /* create a global symbol at interrupts end */
  } > m_interrupts

  /* The program code and other data goes into internal RAM */
  .text :
  {
    . = ALIGN(4);
    __m_text_start = .;      /* create a global symbol at text start */
    *(.text)                 /* .text sections (code) */
    *(.text*)                /* .text* sections (code) */
    *(.rodata)               /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)              /* .rodata* sections (constants, strings, etc.) */
    *(.glue_7)               /* glue arm to thumb code */
    *(.glue_7t)              /* glue thumb to arm code */
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
  } > m_text

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > m_text

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > m_text

 .ctors :
  {
    __CTOR_LIST__ = .;
    /* gcc uses crtbegin.o to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.o; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o
       is in.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    /* We don't want to include the .ctor section from
       from the crtend.o file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    __CTOR_END__ = .;
  } > m_text

  .dtors :
  {
    __DTOR_LIST__ = .;
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __DTOR_END__ = .;
  } > m_text

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > m_text

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > m_text

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    __m_text_end = .;      /* create a global symbol at text end */
  } > m_text

  __etext = .;    /* define a global symbol at end of code */

  .data :
  {
    . = ALIGN(4);
    __DATA_RAM = .;
    __data_start__ = .;      /* create a global symbol at data start */
    __m_data_start = .;      /* create a global symbol at data start */
    *(.rfctrl)               /* .rfctrl section */
    *(.rfctrl*)              /* .rfctrl* section */
    *(.data)                 /* .data sections */
    *(.data*)                /* .data* sections */
    KEEP(*(.jcr*))
    . = ALIGN(4);
    __data_end__ = .;        /* define a global symbol at data end */
  } > m_data

  /* Uninitialized data section */
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    . = ALIGN(4);
    __START_BSS = .;
    __bss_start__ = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
    __END_BSS = .;
  } > m_data

  .heap :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    __HeapBase = .;
    . += HEAP_SIZE;
    __HeapLimit = .;
  } > m_data

    .stack :
  {
    . = ALIGN(8);
    . += STACK_SIZE;
    __m_data_end = .;        /* define a global symbol at data end */
  } > m_data
   /* Initializes stack on the end of block */
  __StackTop   = ORIGIN(m_data) + LENGTH(m_data);
  __StackLimit = __StackTop - STACK_SIZE;
  PROVIDE(__stack = __StackTop);

  /* DTCM Assignment */
  .hshake_rsvd :
  {
    __m_dtcm_start = .;      /* create a global symbol at data start */
	/* Reserved 256 bytes: Used by Bootrom during warm-reset and boot hand-shake */
	. += 0x100;
  } > m_dtcm

  .cal_tbl (NOLOAD):
  {
	. = ALIGN(0x1000);
	__rfic_cal_start = .;
	*(.cal_tbl)
	__rfic_cal_end = .;
        __m_dtcm_end = .;      /* create a global symbol at data end */
  } > m_dtcm

  .ARM.attributes 0 : { *(.ARM.attributes) }

  ASSERT((__rfic_cal_end - __rfic_cal_start) <= 0xC000, "Cal table exceeded 48k")
  /*Does m_data overlaps HIF? */
  ASSERT(__StackTop < 0x1F81C000, "m_data overlaps to HIF")
  ASSERT(__StackLimit >= __HeapLimit, "region m_data overflowed with stack and heap")
}
