/* Linker script to configure memory regions. */
MEMORY
{
  ROM  (rx)  : ORIGIN = 0x00000000, LENGTH = 0x100000   /* 1024k */
  RAM  (rwx) : ORIGIN = 0x10000000, LENGTH = 0x040000   /*  256k */
  RAM2 (rwx) : ORIGIN = 0x20000000, LENGTH = 0x008000   /*   32k */
}

/* Library configurations */
GROUP(libgcc.a libc.a libm.a libnosys.a)

ENTRY(BootLoader)

SECTIONS
{
	. = ORIGIN(ROM);

	.boot :
	{
		KEEP(*(.boot))
	} > ROM

	__eboot = LOADADDR(.boot) + SIZEOF(.boot);

	.text :
	{
		__code_start__ = .;
		KEEP(*(.vectors))
		*(.startup)
		*(.text .text.*)
		KEEP(*(.init))
		KEEP(*(.fini))

		/* .ctors */
		*crtbegin.o(.ctors)
		*crtbegin?.o(.ctors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
		*(SORT(.ctors.*))
		*(.ctors)

		/* .dtors */
		*crtbegin.o(.dtors)
		*crtbegin?.o(.dtors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
		*(SORT(.dtors.*))
		*(.dtors)

		*(.rodata*)

		KEEP(*(.eh_frame*))
		__code_end__ = .;
	} > RAM AT > ROM

	__ecode = LOADADDR(.text) + SIZEOF(.text);

	.ARM.extab :
	{
		__extab_start = .;
		*(.ARM.extab* .gnu.linkonce.armextab.*)
		__extab_end = .;
	} > RAM AT > ROM

	__eextab = LOADADDR(.ARM.extab) + SIZEOF(.ARM.extab);

	.ARM.exidx :
	{
		__exidx_start = .;
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		__exidx_end = .;
	} > RAM AT > ROM

	__etext = LOADADDR(.ARM.exidx) + SIZEOF(.ARM.exidx);

	.data :
	{
		__data_start__ = .;
		*(vtable)
		*(.data*)

		. = ALIGN(4);
		/* preinit data */
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP(*(.preinit_array))
		PROVIDE_HIDDEN (__preinit_array_end = .);

		. = ALIGN(4);
		/* init data */
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		PROVIDE_HIDDEN (__init_array_end = .);

		. = ALIGN(4);
		/* finit data */
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP(*(SORT(.fini_array.*)))
		KEEP(*(.fini_array))
		PROVIDE_HIDDEN (__fini_array_end = .);

		KEEP(*(.jcr*))
		. = ALIGN(4);
		/* All data end */
		__data_end__ = .;
	} > RAM AT > ROM

	__edata = LOADADDR(.data) + SIZEOF(.data);

	.bss (NOLOAD):
	{
		. = ALIGN(4);
		__bss_start__ = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		__bss_end__ = .;
	} > RAM2

	.heap (NOLOAD):
	{
		__end__ = .;
		end = __end__;
		*(.heap*)
		__heap_limit = .;
	} > RAM2

	/* .stack_dummy section doesn't contains any symbols. It is only
	 * used for linker to calculate size of stack sections, and assign
	 * values to stack symbols later */
	.stack_dummy (NOLOAD):
	{
		*(.stack*)
	} > RAM2

	/* Set stack top to end of RAM, and stack limit move down by
	 * size of stack_dummy section */
	__initial_sp  = ORIGIN(RAM2) + LENGTH(RAM2);
	__stack_limit = __initial_sp - SIZEOF(.stack_dummy);
	PROVIDE(__stack = __initial_sp);

	/* Check if data + heap + stack exceeds RAM limit */
	ASSERT(__stack_limit >= __heap_limit, "region RAM2 overflowed with stack")
}
