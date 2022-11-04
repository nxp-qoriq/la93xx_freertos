
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

ENTRY(m4_startup)

	STACK_SIZE = 0xC00;
	HEAP_SIZE = 0x200;

MEMORY
{
	rom (rx) : org = 0x1F800000, len = 0x00001000
	TCM (rw) : org = 0x2000F400, len = 0xC00
}

SECTIONS
{
	.text :
	{
		. = ALIGN(4);
		bin/start.o
		*(.text)	/* Program code */
		. = ALIGN(0x4);
	} > rom

	__end_text = . ;

	.rodata    :
	{
		*(.rodata)
	} > rom

	.data    :
	{
		*(.data)
	} > rom

	. = .;
	__bss_start = .;
	.bss (NOLOAD)       :
	{
		*(.bss)
	} > rom
	__bss_end = .;

	.stack :
	{
		. = ALIGN(4);
		. += STACK_SIZE;
	} > TCM

	/* Initializes stack on the end of block */
	__StackTop   = ORIGIN(TCM) + LENGTH(TCM);
	__StackLimit = __StackTop - STACK_SIZE;
	PROVIDE(__stack = __StackTop);
	__end = .;
}
