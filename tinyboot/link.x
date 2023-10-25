MEMORY {
    PROGMEM (rwx): ORIGIN = 0x8000, LENGTH = 512
    RAM (rw): ORIGIN = 0x0000, LENGTH = 32K
}

EXTERN(__start);
ENTRY(__start);

SECTIONS {
    PROVIDE(__stack_start = ORIGIN(RAM) + LENGTH(RAM));

    PROVIDE(__stext = ORIGIN(PROGMEM));

    .text __stext : {
        *(.start);

        *(.text .text.*);

        . = ALIGN(4);
        __etext = .;
    } > PROGMEM

    .rodata : ALIGN(4) {
        . = ALIGN(4);
        __srodata = .;
        *(.rodata .rodata.*);
        . = ALIGN(4);
        __erodata = .;
    } > PROGMEM

    /DISCARD/ : {
        /* throw away RAM sections to get a link error if they're used. */
        *(.bss);
        *(.bss.*);
        *(.data);
        *(.data.*);
        *(COMMON);
        *(.ARM.exidx);
        *(.ARM.exidx.*);
        *(.ARM.extab.*);
        *(.got);
    }
}
